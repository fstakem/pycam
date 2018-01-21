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
pip install rshell
rshell --buffer-size = 30 -p /dev/ttyUSB0

rshell --port /dev/ttyUSB0 --baud 115200 --buffer-size 128 --editor nano
sudo ./pycam_repl_env/bin/rshell --port /dev/ttyUSB0 --baud 115200 --buffer-size 128 --editor nano
