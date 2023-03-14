#!/bin/bash

rm .config
cp spider_usb.config .config
make clean
make -j
sudo service klipper stop
make flash FLASH_DEVICE=/dev/serial/by-id/usb-Klipper_stm32f446xx_1F0028000550305538333620-if00 
sudo service klipper start
