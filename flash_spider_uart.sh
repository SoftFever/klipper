#!/bin/bash

sudo service klipper stop
make clean
make -j
./scripts/flash-sdcard.sh /dev/ttyAMA0 fysetc-spider-v1
sudo service klipper start