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

# Run
rshell --port /dev/ttyUSB0 --baud 115200 --buffer-size 128 --editor nano
sudo ./pycam_repl_env/bin/rshell --port /dev/ttyUSB0 --baud 115200 --buffer-size 128 --editor vim

# Copy files
cp ./pycam/arducam.py /pyboard/arducam.py
cp ./pycam/arducam_constants.py /pyboard/arducam_constants.py
cp ./pycam/main.py /pyboard/main.py

