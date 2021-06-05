import serial
import argparse

def flushBuffer(ser):
    ser.flushInput()
    ser.flushOutput()

def writeBuffer(ser, message):
    ser.write(message.encode("utf-8"))

def readBuffer(ser):
    while True:
        print(ser.readline()) 

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Working with the serial port.")

    parser.add_argument('-f',
                        '--functionality',
                        metavar='FUNCTIONALITY',
                        type=str,
                        help='desired functionality',
                        choices = ['FLUSH', 'WRITE', 'READ'],
                        required=True)

    parser.add_argument('-s',
                        '--serial',
                        metavar='SERIAL',
                        type=str,
                        help='serial device',
                        required=True)

    parser.add_argument('-m',
                        '--message',
                        metavar='MESSAGE',
                        type=str,
                        help='message',
                        required=False)

    args = parser.parse_args()
    function = args.functionality
    device = args.serial
    message = args.message

    ser = serial.Serial(device)

    if function == 'FLUSH':
        flushBuffer(ser)
    elif function == 'WRITE':
        writeBuffer(ser, message)
    elif function == 'READ':
        readBuffer(ser) 




