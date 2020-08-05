#!/bin/bash

# Run this before running bbb_canbus_listener
config-pin p9.19 can
config-pin p9.20 can

sudo ip link set can0 up type can bitrate 500000
sudo ifconfig can0 up

config-pin p9.11 uart
config-pin p9.13 uart

config-pin p9.24 uart
config-pin p9.26 uart

# print status of pins
config-pin -q P9.19
config-pin -q P9.20
config-pin -q P9.11
config-pin -q P9.13
config-pin -q P9.24
config-pin -q P9.26
