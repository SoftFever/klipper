sudo apt install -y \
    virtualenv python2-dev libffi-dev build-essential \
    libncurses-dev libusb-dev  avrdude gcc-avr binutils-avr \
    avr-libc stm32flash binutils-multiarch libusb-1.0-0 \
    dfu-util unzip git
sudo usermod -a -G dialout $USER

