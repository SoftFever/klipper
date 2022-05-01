#!/bin/bash

rm .config
cp octopus_uart.config .config
make clean
make -j
# sudo service klipper stop
# make flash FLASH_DEVICE=/dev/ttyAMA0
# sudo service klipper start
