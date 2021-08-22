from nt_connection.Connection import Connection
import generated_python.Value_pb2 as Value_pb2
import generated_python.PowerController_pb2 as PowerController_pb2
import generated_python.Node_pb2 as Node_pb2
import requests
import threading
import time
import sys
from nt_connection import uri


def main():
    print("Connecting to the network table server")
    nt_connection = Connection()
    nt_connection.Connect()

    power_data = PowerController_pb2.PowerController()
    value = Value_pb2.Value()
    value.type = Value_pb2.Value.Type.INT
    value.int_data = 1

    power_uri = uri.POWER_CONTROLLER

    print(value)
    values = {power_uri: value}

    nt_connection.setValues(values)
    nt_connection.Disconnect()


if __name__ == "__main__":
    main()
