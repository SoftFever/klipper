#!/bin/bash

rm .config
cp octopus_uart.config .config
make clean
make -j
sudo service klipper stop
# This only copies the firmware to sd card.
# need to power off then power on the machine(MCU) to trigger the actual flash.(SDIO limitation)
./scripts/flash-sdcard.sh /dev/ttyAMA0 btt-octopus-pro
sudo service klipper start
