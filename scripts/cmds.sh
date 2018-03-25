#!/usr/bin/env bash

# Find port
dmesg | grep ttyUSB

ll /run/lock/


# Minicom
# Connect to board from cli
sudo minicom -D /dev/ttyUSB0

# Reconnet to board
sudo minicom -S ttyusb0 -o

# Exit minicom
Ctrl-a x

# Rshell
# Another program to connect to board

# Install
pip install rshell

# Run: pyboard shows up on ttyACM0
rshell --port /dev/ttyUSB0 --baud 115200 --buffer-size 128 --editor nano
sudo ~/envs/upy_env/bin/rshell --port /dev/ttyUSB0 --baud 115200 --buffer-size 128 --editor vim

# Copy files
cp ./pycam/arducam.py /pyboard/arducam.py
cp ./pycam/arducam_constants.py /pyboard/arducam_constants.py
cp ./pycam/main.py /pyboard/main.py

# Flash new firmware with esptool
sudo ~/envs/upy_env/bin/esptool.py --port /dev/ttyUSB0 erase_flash
sudo ~/envs/upy_env/bin/esptool.py --chip esp32 -p /dev/ttyUSB0 write_flash -z 0x1000 <firmmware_path>
sudo ~/envs/upy_env/bin/esptool.py --chip esp8266 --p /dev/ttyUSB0 write_flash --flash_size=detect 0 <firmmware_path>


