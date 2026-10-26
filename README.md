# Умный корректор осанки (dfp2026-b01-508)

Носимое устройство для поддержания правильной осанки с поэтапной реализацией:
пассивный мониторинг (IMU + BLE + вибро) -> активная механическая коррекция.

## Команда
* Стефанович Ярослав Андреевич (Б04-508)
* Пыханов Иван (Б04-507)
* Ментор: Алтынов Алексей Владимирович

## Материалы проекта
* [Проектное предложение (PDF)](docs/APP.pdf)
* [Спецификация компонентов (BOM)](bom/BOM.md)

## Актуальная версия — 26 октября 2026

- [Прошивка v0.1.1](firmware/README.md): ESP32-C3 SuperMini + MPU6050/MPU6500.
- [Корпус v3](hardware/case-v3/README.txt): исходники OpenSCAD.
- [Проверки и следующие шаги](docs/STATUS.md).

## Сборка и проверки

Для прошивки установите Arduino CLI и пакет esp32 by Espressif Systems версии 3.3.2. Команды выполняются из корня репозитория; они не прошивают устройство.

```sh
arduino-cli core update-index --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core install esp32:esp32@3.3.2 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc,FlashSize=4M,PartitionScheme=huge_app,UploadSpeed=921600 firmware/SpineSmart
```

Тест логики на компьютере (C++17) и экспорт корпуса (OpenSCAD 2021.01):

```sh
mkdir -p build
g++ -std=c++17 -O2 -Wall -Wextra -I firmware/SpineSmart tests/logic-selftest.cpp -o build/logic-selftest
./build/logic-selftest
openscad -o build/bottom_v3.stl hardware/case-v3/bottom_v3.scad
openscad -o build/lid_v3.stl hardware/case-v3/lid_v3.scad
```

Полученные STL предназначены для первой примерки; программная проверка геометрии не подтверждает физическую посадку.
