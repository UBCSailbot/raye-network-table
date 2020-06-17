#!/usr/bin/python3
# Script to mock UCCM sensor data on a virtual CAN bus on vcan0
# Sends garbage data but you specify the SID
# --> needs virtual can set up first -> ./setup_vcan.shi
# needs server ->  in build/bin -> ./server &
# You should run this as a background job -> python3 mock_sensors.py &
import time
import can
import sys
import argparse


def send_sensor_data(device, SID):
    bus = can.interface.Bus(bustype='socketcan', channel='vcan0', bitrate=250000)
    print("Sending CAN messages on {}".format(bus.channel_info))
    print("device = {} \nRTR: 0b0\nDLC: 8\n".format(device))
    if device == 'bms':
        while True:
            msg = can.Message(arbitration_id=SID[device][i], dlc=0x8, data=[20,20,20,20,20,20,20,20])
            try:
                bus.send(msg)
            except can.CanError:
                print("Error Sending on CAN bus")
                pass
            time.sleep(3)
    elif device == 'gps':
        while True:

            for i in range(len(SID[device])):
                msg = can.Message(arbitration_id=SID[device][i], dlc=0x8, data=[50,50,50,50,50,50,50,50])
                try:
                    bus.send(msg)
                except can.CanError:
                    print("Error Sending on CAN bus")
                    pass
             
    elif device == 'sail':
        while True:
            # cycles through all of the CAN ID frames for the gps
            for i in range(len(SID[device])):
                msg = can.Message(arbitration_id=SID[device][i], dlc=0x8, data=[40,40,40,40,40,40,40,40])
                try:
                    bus.send(msg)
                except can.CanError:
                    print("Error Sending on CAN bus")
                    pass
    
    elif device == 'wind':
        while True:
            for i in range(len(SID[device])):
                msg = can.Message(arbitration_id=SID[device][i], dlc=0x8, data=[30,30,30,30,30,30,30,30])
                try:
                    bus.send(msg)
                except can.CanError:
                    print("Error Sending on CAN bus")
                    pass
    elif device == 'acc':
        while True:
            msg = can.Message(arbitration_id=SID[device][i], dlc=0x8, data=[10,10,10,10,10,10,10,10])
            try:
                bus.send(msg)
            except can.CanError:
                print("Error Sending on CAN bus")
                pass
    else: # if device == all
        while True:
            for i in range(len(SID[device])):
                msg = can.Message(arbitration_id=SID[device][i], dlc=0x8, data=[1,1,1,1,1,1,1,1])
                try:
                    bus.send(msg)
                except can.CanError:
                    print("Error Sending on CAN bus")
                    pass
def main():
    # SID is taken from frame_parser.h data relating to the CAN ID of each device
    SID = {'bms': [0x08], 
           'gps': [0x11, 0x100,0x101,0x110],
           'sail': [0x0F],
           'wind': [0x0, 0x10, 0x111],
           'acc': [0xAC],
           'all': [0x08, 0x11, 0x100, 0x101, 
                   0x110, 0x0F, 0x0, 0x10, 
                   0x111, 0xAC]}
    parser = argparse.ArgumentParser(description="Sensor mocking")
    parser.add_argument("-d", "--device_id", help="device to simulate. Options: gps, bms, wind, acc", default='all')
    args = parser.parse_args()

    if args.device_id in SID:
        send_sensor_data(args.device_id, SID)
    else:
        print("device not found. Exiting...")

    return

if __name__ == "__main__":
    main()
