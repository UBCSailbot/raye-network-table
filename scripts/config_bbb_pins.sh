#!/bin/bash

# Configure UART pins
config-pin p9.11 uart
config-pin p9.13 uart
config-pin p9.21 uart
config-pin p9.22 uart

# Configure CAN pins
config-pin p9.19 can
config-pin p9.20 can

exit 0
