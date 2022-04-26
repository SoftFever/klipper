#!/bin/bash

rm .config
cp octopus.config .config
make clean
make -j
sudo service klipper stop
make flash FLASH_DEVICE=/dev/serial/by-id/usb-Klipper_stm32f446xx_0E000C00145053424E363620-if00
sudo service klipper start