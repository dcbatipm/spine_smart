#include <cstdio>
#include <cmath>
#include "PostureLogic.h"
constexpr float PI=3.14159265358979323846f;
static float dot3(const float*a,const float*b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
int main(){
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

std::printf("Logic selftest %d/%d\n",passed,total);return passed==total?0:1;}
