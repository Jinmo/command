#!/bin/bash

sudo systemctl stop bluetooth
sudo apt-get update
sudo apt-get install libusb-dev libdbus-1-dev libglib2.0-dev libudev-dev libical-dev libreadline-dev libdbus-glib-1-dev unzip python3-pip
pushd /tmp
mkdir bluez
cd bluez
wget http://www.kernel.org/pub/linux/bluetooth/bluez-5.44.tar.xz
tar xf bluez-5.44.tar.xz
cd bluez-5.44
./configure --prefix=/usr --sysconfdir=/etc --localstatedir=/var --enable-library
make
sudo make install
sudo ln -svf /usr/libexec/bluetooth/bluetoothd /usr/sbin/
sudo install -v -dm755 /etc/bluetooth
sudo install -v -m644 src/main.conf /etc/bluetooth/main.conf
sudo systemctl daemon-reload
sudo systemctl start bluetooth
bluetoothd --version  should now print 5.44
sudo apt-get install python3-dbus
popd
sudo python3 -m pip install -r requirements.txt
