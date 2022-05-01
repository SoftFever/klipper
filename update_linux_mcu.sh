#!/bin/bash

rm .config
cp linux_mcu.config .config
make clean
make -j
sudo service klipper stop
make flash
sudo service klipper start
