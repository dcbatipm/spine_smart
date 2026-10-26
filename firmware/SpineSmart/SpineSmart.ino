#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <atomic>
#include "PostureLogic.h"

static const char *VERSION = "0.1.1";
static const char *SERVICE_UUID = "fa5e0001-3a93-4e9c-a5bc-5350494e4501";
static const char *STATUS_UUID = "fa5e0002-3a93-4e9c-a5bc-5350494e4501";
static const char *ANGLE_UUID = "fa5e0003-3a93-4e9c-a5bc-5350494e4501";
constexpr int SDA_PIN = 8, SCL_PIN = 9, MOTOR_PIN = 3;
constexpr uint32_t SAMPLE_MS = 20, REPORT_MS = 1000, RETRY_MS = 3000;
constexpr uint32_t ALERT_COOLDOWN_MS = 30000, MOTOR_PULSE_MS = 150;
constexpr uint16_t CAL_SAMPLES = 150;

Preferences prefs;
PostureLogic posture;
BLECharacteristic *bleStatus = nullptr, *bleAngle = nullptr;
std::atomic<bool> bleConnected(false), restartAdvertising(false);
bool sensorPresent = false, calibrated = false, calibrating = false;
bool filterReady = false, still = false, motorArmed = false, motorActive = false;
bool hasAlerted = false, streamEnabled = true, manualPulse = false;
uint8_t imuAddress = 0;
uint8_t imuIdentity = 0;
float gyroBias[3]={0,0,0};
uint32_t lastSample = 0, lastReport = 0, lastProbe = 0, lastAlert = 0;
uint32_t calStarted = 0, motorStarted = 0, readErrors = 0, alertCount = 0;
uint16_t calCount = 0;
float referenceV[3] = {0,0,1}, filtered[3] = {0,0,1}, calSum[3] = {0,0,0};
float calFirst[3] = {0,0,1}, angleDeg = NAN, gyroDps = NAN, accelerationG = NAN;
float lastAccel[3]={0,0,0}, lastGyro[3]={0,0,0};
uint32_t sampleCount=0;
char deviceName[32];
char commandLine[128];
size_t commandLength = 0;
bool commandOverflow = false;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { bleConnected.store(true); }
  void onDisconnect(BLEServer*) override {
    bleConnected.store(false); restartAdvertising.store(true);
  }
};

float norm3(const float *v) { return sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); }
float dot3(const float *a,const float *b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
void stopMotor() { digitalWrite(MOTOR_PIN,LOW); motorActive=false; manualPulse=false; }
void pulseMotor(uint32_t now, bool manual) {
  if (!motorArmed || motorActive) return;
  motorStarted=now; motorActive=true; manualPulse=manual; digitalWrite(MOTOR_PIN,HIGH);
}
void say(const char *s) { if (Serial) Serial.println(s); }

const char *stateName() {
  if (!sensorPresent) return "WAIT_SENSOR";
  if (calibrating) return "CALIBRATING";
  if (!calibrated) return "NEED_CALIBRATION";
  if (!still) return "MOVING";
  if (posture.slouch) return "SLOUCH";
  if (posture.pending) return "TILT_PENDING";
  return "UPRIGHT";
}

bool writeReg(uint8_t reg,uint8_t value) {
  Wire.beginTransmission(imuAddress); Wire.write(reg); Wire.write(value);
  return Wire.endTransmission()==0;
}
bool readRegs(uint8_t reg,uint8_t *data,size_t count) {
  Wire.beginTransmission(imuAddress); Wire.write(reg);
  if (Wire.endTransmission(false)!=0) return false;
  size_t got=Wire.requestFrom(imuAddress,count,true);
  if (got!=count) { while(Wire.available()) Wire.read(); return false; }
  for(size_t i=0;i<count;i++) data[i]=Wire.read();
  return true;
}
void sensorLost() {
  sensorPresent=false; filterReady=false; still=false; angleDeg=NAN;
  calibrating=false; calCount=0; posture.reset(); stopMotor();
}
void probeSensor() {
  for(uint8_t addr : {uint8_t(0x68),uint8_t(0x69)}) {
    imuAddress=addr;
    // Address-only probe treats an absent sensor as normal, without bus-error spam.
    Wire.beginTransmission(imuAddress);
    if (Wire.endTransmission()!=0) continue;
    uint8_t who=0;
    if (!readRegs(0x75,&who,1) || (who!=0x68 && who!=0x70)) continue;
    // MPU6050 / MPU6500: 50 Hz samples, +/-2g, +/-250 dps.
    if (!writeReg(0x6B,0x01)) continue;
    delay(30);
    if (!writeReg(0x6C,0x00)||!writeReg(0x1A,0x03)||!writeReg(0x19,19)||
        !writeReg(0x1B,0x00)||!writeReg(0x1C,0x00)) continue;
    // MPU6500 has a separate accelerometer low-pass register (41 Hz).
    if(who==0x70 && !writeReg(0x1D,0x03)) continue;
    imuIdentity=who;
    gyroBias[0]=gyroBias[1]=gyroBias[2]=0;
    if(prefs.getUInt("gyro_id",0)==who && prefs.getBytesLength("gyro_bias")==sizeof(gyroBias)) {
      prefs.getBytes("gyro_bias",gyroBias,sizeof(gyroBias));
      if(!isfinite(norm3(gyroBias)) || norm3(gyroBias)>15) gyroBias[0]=gyroBias[1]=gyroBias[2]=0;
    }
    sensorPresent=true; filterReady=false; posture.reset();
    say(who==0x70?"OK SENSOR_CONNECTED MPU6500":"OK SENSOR_CONNECTED MPU6050"); return;
  }
  sensorLost();
}
void resetCalSamples() { calCount=0; calSum[0]=calSum[1]=calSum[2]=0; }
void zeroGyro() {
  if(!sensorPresent) {say("ERR SENSOR_NOT_CONNECTED");return;}
  stopMotor();posture.reset();calibrating=false;
  float sum[3]={0,0,0},firstG[3]={0,0,0},firstA[3]={0,0,1},firstNorm=1;
  say("OK GYRO_ZERO_START keep sensor stationary for 3 seconds; posture reference unchanged");
  for(int n=0;n<150;n++) {
    delay(20);
    uint8_t b[14];
    if(!readRegs(0x3B,b,sizeof(b))) {readErrors++;sensorLost();say("ERR GYRO_ZERO_READ");return;}
    float a[3],g[3];
    for(int i=0;i<3;i++) {
      a[i]=int16_t((uint16_t(b[2*i])<<8)|b[2*i+1])/16384.0f;
      g[i]=int16_t((uint16_t(b[8+2*i])<<8)|b[9+2*i])/131.0f;
    }
    float an=norm3(a);
    if(!isfinite(an)||an<0.85f||an>1.15f||norm3(g)>15) {say("ERR GYRO_ZERO_UNSTABLE");return;}
    for(int i=0;i<3;i++)a[i]/=an;
    if(n==0) {firstNorm=an;for(int i=0;i<3;i++){firstA[i]=a[i];firstG[i]=g[i];}}
    if(dot3(a,firstA)<0.99939f||fabsf(an-firstNorm)>0.04f) {say("ERR GYRO_ZERO_MOVEMENT");return;}
    for(int i=0;i<3;i++) {
      if(fabsf(g[i]-firstG[i])>0.75f) {say("ERR GYRO_ZERO_MOVEMENT");return;}
      sum[i]+=g[i];
    }
  }
  for(int i=0;i<3;i++)sum[i]/=150;
  if(prefs.putBytes("gyro_bias",sum,sizeof(sum))!=sizeof(sum)||prefs.putUInt("gyro_id",imuIdentity)!=sizeof(uint32_t)) {
    say("ERR GYRO_ZERO_SAVE");return;
  }
  for(int i=0;i<3;i++)gyroBias[i]=sum[i];
  if(Serial)Serial.printf("OK GYRO_ZERO_SAVED %.5f %.5f %.5f dps\n",gyroBias[0],gyroBias[1],gyroBias[2]);
}
void persistSettings() {
  prefs.putFloat("threshold",posture.threshold);
  prefs.putUInt("holdms",posture.holdMs);
}
void sampleSensor(uint32_t now) {
  uint8_t b[14];
  if (!readRegs(0x3B,b,sizeof(b))) { readErrors++; sensorLost(); return; }
  float a[3],g[3];
  for(int i=0;i<3;i++) {
    a[i]=int16_t((uint16_t(b[2*i])<<8)|b[2*i+1])/16384.0f;
    g[i]=int16_t((uint16_t(b[8+2*i])<<8)|b[9+2*i])/131.0f-gyroBias[i];
  }
  for(int i=0;i<3;i++) {lastAccel[i]=a[i];lastGyro[i]=g[i];}
  sampleCount++;
  accelerationG=norm3(a); gyroDps=norm3(g);
  if (!isfinite(accelerationG)||accelerationG<0.1f) {
    still=false; posture.reset(); angleDeg=NAN; stopMotor(); return;
  }
  float unit[3]; for(int i=0;i<3;i++) unit[i]=a[i]/accelerationG;
  still=accelerationG>0.85f&&accelerationG<1.15f&&gyroDps<15.0f;
  if (!filterReady) { for(int i=0;i<3;i++) filtered[i]=unit[i]; filterReady=true; }
  else for(int i=0;i<3;i++) filtered[i]+=0.06f*(unit[i]-filtered[i]);
  if (calibrating) {
    // Direction stability also gates calibration; allow measured static gain error.
    const bool veryStill=accelerationG>0.85f&&accelerationG<1.15f&&gyroDps<3.0f;
    if (!veryStill || (calCount>0&&dot3(unit,calFirst)<0.99863f)) resetCalSamples();
    else {
      if(calCount==0) for(int i=0;i<3;i++) calFirst[i]=unit[i];
      for(int i=0;i<3;i++) calSum[i]+=unit[i];
      calCount++;
      if(calCount>=CAL_SAMPLES) {
        float n=norm3(calSum);
        for(int i=0;i<3;i++) { referenceV[i]=calSum[i]/n; filtered[i]=referenceV[i]; }
        prefs.putBytes("reference",referenceV,sizeof(referenceV));
        calibrated=true; calibrating=false; posture.reset();
        say("OK CALIBRATED saved; reference is current mounting orientation");
      }
    }
    if(calibrating&&uint32_t(now-calStarted)>15000) {
      calibrating=false; resetCalSamples(); say("ERR CALIBRATION_TIMEOUT hold still for 3 seconds");
    }
  }
  const float fn=norm3(filtered);
  if(calibrated&&fn>0.1f) {
    float cosine=dot3(filtered,referenceV)/fn;
    angleDeg=acosf(constrain(cosine,-1.0f,1.0f))*180.0f/PI;
  } else angleDeg=NAN;
  posture.update(angleDeg,calibrated&&!calibrating&&still,now);
  if(!manualPulse&&(!still||calibrating||!calibrated||!posture.pending)) stopMotor();
  if(posture.slouch&&(!hasAlerted||uint32_t(now-lastAlert)>=ALERT_COOLDOWN_MS)) {
    hasAlerted=true; lastAlert=now; alertCount++;
    pulseMotor(now,false); say("EVENT POSTURE_ALERT");
  }
}

void report() {
  char angleText[20],line[480];
  if(isfinite(angleDeg)) snprintf(angleText,sizeof(angleText),"%.2f",angleDeg);
  else strcpy(angleText,"null");
  snprintf(line,sizeof(line),
    "{\"device\":\"%s\",\"fw\":\"%s\",\"uptime_ms\":%lu,\"state\":\"%s\","
    "\"sensor\":%s,\"calibrated\":%s,\"angle_deg\":%s,\"threshold_deg\":%.1f,"
    "\"delay_ms\":%lu,\"motor_armed\":%s,\"motor_active\":%s,\"ble_connected\":%s,"
    "\"cal_samples\":%u,\"read_errors\":%lu,\"alerts\":%lu,\"free_heap\":%u}",
    deviceName,VERSION,(unsigned long)millis(),stateName(),sensorPresent?"true":"false",
    calibrated?"true":"false",angleText,posture.threshold,(unsigned long)posture.holdMs,
    motorArmed?"true":"false",motorActive?"true":"false",bleConnected.load()?"true":"false",
    calCount,(unsigned long)readErrors,(unsigned long)alertCount,ESP.getFreeHeap());
  say(line);
}
void publishBLE() {
  // Read-only over radio. No unauthenticated remote motor/calibration commands.
  bleStatus->setValue(stateName());
  bleAngle->setValue(reinterpret_cast<uint8_t*>(&angleDeg),sizeof(angleDeg));
  if(bleConnected.load()) { bleStatus->notify(); bleAngle->notify(); }
}
void runSelfTest() {
  PostureLogic p; int passed=0,total=0;
  auto check=[&](bool value){total++; if(value)passed++;};
  check(!p.update(20,true,100)&&p.pending&&!p.slouch);
  check(!p.update(20,true,10099));
  check(p.update(20,true,10100)&&p.slouch);
  check(!p.update(20,true,11000)); // one transition, no retrigger each sample
  check(!p.update(13,true,11001)&&p.slouch); // hysteresis
  check(!p.update(11,true,11002)&&!p.pending&&!p.slouch);
  p.update(20,true,12000); p.update(20,false,20000); check(!p.pending);
  p.update(20,true,21000); p.update(NAN,true,22000); check(!p.pending);
  p.reset(); p.update(20,true,UINT32_MAX-5000); check(p.update(20,true,4999));
  float u[3]={0,0,1},v[3]={0,0,1};check(fabsf(acosf(dot3(u,v)))<0.001f);
  v[2]=0;v[0]=1;check(fabsf(acosf(dot3(u,v))*180/PI-90)<0.001f);
  if(Serial) Serial.printf("SELFTEST %s %d/%d (logic only, no sensor or motor exercised)\n",passed==total?"PASS":"FAIL",passed,total);
}

void handleCommand(String cmd) {
  cmd.trim(); cmd.toLowerCase();
  if(cmd=="status") { report(); return; }
  if(cmd=="zero gyro") {zeroGyro();return;}
  if(cmd=="bias") {
    if(Serial)Serial.printf("GYRO_BIAS %.5f %.5f %.5f dps\n",gyroBias[0],gyroBias[1],gyroBias[2]);
    return;
  }
  if(cmd=="bus") {
    stopMotor();
    if(Serial) Serial.printf("BUS SDA=8 level=%d SCL=9 level=%d\n",digitalRead(SDA_PIN),digitalRead(SCL_PIN));
    unsigned found=0;
    for(uint8_t addr=1;addr<127;addr++) {
      Wire.beginTransmission(addr);
      uint8_t result=Wire.endTransmission();
      if(result==0) {
        found++;
        if(Serial) Serial.printf("BUS ACK 0x%02X\n",addr);
        if(addr==0x68||addr==0x69) {
          uint8_t saved=imuAddress, who=0;imuAddress=addr;
          bool ok=readRegs(0x75,&who,1);imuAddress=saved;
          if(Serial) Serial.printf("BUS WHO_AM_I read=%s value=0x%02X\n",ok?"ok":"failed",who);
        }
      }
    }
    if(Serial) Serial.printf("BUS DONE devices=%u\n",found);
    return;
  }
  if(cmd=="diag") {
    if(!sensorPresent) {say("ERR SENSOR_NOT_CONNECTED");return;}
    if(Serial) Serial.printf("{\"imu_address\":%u,\"samples\":%lu,\"accel_g\":[%.5f,%.5f,%.5f],\"gyro_dps\":[%.4f,%.4f,%.4f],\"accel_norm_g\":%.5f,\"gyro_norm_dps\":%.4f,\"still\":%s}\n",imuAddress,(unsigned long)sampleCount,lastAccel[0],lastAccel[1],lastAccel[2],lastGyro[0],lastGyro[1],lastGyro[2],accelerationG,gyroDps,still?"true":"false");
    return;
  }
  if(cmd=="help") {
    say("COMMANDS: status | diag | bus | zero gyro | bias | help | selftest | stream on/off | calibrate | clearcal | threshold 5..45 | delay 2..60 | motor on/off/test | reboot");return;
  }
  if(cmd=="selftest") { runSelfTest(); return; }
  if(cmd=="stream on"||cmd=="stream off") {streamEnabled=cmd.endsWith("on");say("OK STREAM");return;}
  if(cmd=="calibrate") {
    if(!sensorPresent) {say("ERR SENSOR_NOT_CONNECTED");return;}
    stopMotor(); posture.reset(); resetCalSamples(); calStarted=millis();calibrating=true;
    say("OK CALIBRATING hold upright and still for 3 seconds");return;
  }
  if(cmd=="clearcal") {
    stopMotor(); calibrated=false;calibrating=false;angleDeg=NAN;posture.reset();
    prefs.remove("reference");say("OK CALIBRATION_CLEARED");return;
  }
  if(cmd=="motor on") {motorArmed=true;say("OK MOTOR_ARMED until reboot");return;}
  if(cmd=="motor off") {motorArmed=false;stopMotor();say("OK MOTOR_DISARMED");return;}
  if(cmd=="motor test") {
    if(!motorArmed) {say("ERR MOTOR_DISARMED");return;}
    pulseMotor(millis(),true);say("OK MOTOR_TEST 150ms");return;
  }
  if(cmd.startsWith("threshold ")||cmd.startsWith("delay ")) {
    bool isThreshold=cmd.startsWith("threshold ");
    String number=cmd.substring(isThreshold?10:6);char *end=nullptr;
    float v=strtof(number.c_str(),&end);
    if(end==number.c_str()||*end!='\0'||!isfinite(v)||v<(isThreshold?5:2)||v>(isThreshold?45:60)) {
      say("ERR RANGE threshold=5..45deg delay=2..60sec");return;
    }
    if(isThreshold)posture.threshold=v;else posture.holdMs=uint32_t(v*1000);
    posture.reset();stopMotor();persistSettings();say("OK SETTINGS_SAVED");return;
  }
  if(cmd=="reboot") {stopMotor();say("OK REBOOT");delay(100);ESP.restart();return;}
  say("ERR UNKNOWN_COMMAND use help");
}
void readCommands() {
  // Limit work per loop so USB floods cannot prevent motor timeout handling.
  for(int n=0;n<128&&Serial.available();n++) {
    char c=Serial.read();
    if(c=='\r')continue;
    if(c=='\n') {
      if(commandOverflow) say("ERR COMMAND_TOO_LONG");
      else if(commandLength) {commandLine[commandLength]='\0';handleCommand(String(commandLine));}
      commandLength=0;commandOverflow=false;
    } else if(!commandOverflow) {
      if(commandLength<sizeof(commandLine)-1) commandLine[commandLength++]=c;
      else commandOverflow=true;
    }
  }
}
void setup() {
  digitalWrite(MOTOR_PIN,LOW);pinMode(MOTOR_PIN,OUTPUT);
  Serial.begin(115200); // Never wait for a USB host: standalone boot must work too.
  prefs.begin("spine-v1",false);
  posture.threshold=prefs.getFloat("threshold",15.0f);
  posture.holdMs=prefs.getUInt("holdms",10000);
  if(!isfinite(posture.threshold)||posture.threshold<5||posture.threshold>45)posture.threshold=15;
  if(posture.holdMs<2000||posture.holdMs>60000)posture.holdMs=10000;
  if(prefs.getBytesLength("reference")==sizeof(referenceV)) {
    prefs.getBytes("reference",referenceV,sizeof(referenceV));
    float n=norm3(referenceV);calibrated=isfinite(n)&&n>0.9f&&n<1.1f;
    if(calibrated)for(int i=0;i<3;i++)referenceV[i]/=n;
  }
  Wire.begin(SDA_PIN,SCL_PIN,100000);Wire.setTimeOut(10);
  // GPIO8 is also the LED on many SuperMini boards: do not blink it while using I2C.
  uint64_t mac=ESP.getEfuseMac();snprintf(deviceName,sizeof(deviceName),"SpineSmart-%04X",unsigned((mac>>32)&0xffff));
  BLEDevice::init(deviceName);
  BLEServer *server=BLEDevice::createServer();server->setCallbacks(new ServerCallbacks());
  BLEService *service=server->createService(SERVICE_UUID);
  bleStatus=service->createCharacteristic(STATUS_UUID,BLECharacteristic::PROPERTY_READ|BLECharacteristic::PROPERTY_NOTIFY);
  bleAngle=service->createCharacteristic(ANGLE_UUID,BLECharacteristic::PROPERTY_READ|BLECharacteristic::PROPERTY_NOTIFY);
  bleStatus->addDescriptor(new BLE2902());bleAngle->addDescriptor(new BLE2902());
  service->start();
  BLEAdvertising *adv=BLEDevice::getAdvertising();adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);adv->start();
  probeSensor();lastProbe=millis();
  say("BOOT SpineSmart 0.1.1; USB 115200; motor OFF; send help");
  publishBLE();report();
}
void loop() {
  uint32_t now=millis();
  if(motorActive&&uint32_t(now-motorStarted)>=MOTOR_PULSE_MS)stopMotor();
  readCommands();
  if(restartAdvertising.exchange(false)) BLEDevice::startAdvertising();
  now=millis();
  if(!sensorPresent&&uint32_t(now-lastProbe)>=RETRY_MS) {lastProbe=now;probeSensor();}
  if(sensorPresent&&uint32_t(now-lastSample)>=SAMPLE_MS) {lastSample=now;sampleSensor(now);}
  if(uint32_t(now-lastReport)>=REPORT_MS) {
    lastReport=now;publishBLE();if(streamEnabled)report();
  }
  delay(1);
}
