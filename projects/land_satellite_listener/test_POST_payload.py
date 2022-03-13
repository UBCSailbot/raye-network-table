"""
This file is intended to verify parsing of the serial protobuf data and its
subsequent placement in the network table does not throw exceptions.
"""
from nt_connection.Connection import Connection
from nt_connection.Help import Help
import generated_python.Satellite_pb2 as Satellite_pb2

sat = Satellite_pb2.Satellite()
helper = Help()
nt_connection = Connection()
nt_connection.Connect()

body0_big_uccm = b'imei=300234068129370&device_type=ROCKBLOCK&serial=16967&momsn=762&transmit_time=22-03-13%2019%3A07%3A06&iridium_latitude=49.2830&iridium_longitude=-123.2476&iridium_cep=3.0&iridium_session_status=0&data=08011abb010a0a080c1006182022024f4e120a08041005180422024f4e1a0a08041005180422024f4e220a08041005180422024f4e2a0a08041005180422024f4e320a08041005182822024f4e3a0a08041005182822024f4e420a08041005182822024f4e52005a00620a08041005180222024f4e6a0a08041005180222024f4e720a08041005180222024f4e7a0a08041005180222024f4e82010a08041005180222024f4e8a010a08041005180222024f4e92010a08041005180222024f4e'  # noqa: E501
body1_sensors = b'imei=300234068129370&device_type=ROCKBLOCK&serial=16967&momsn=761&transmit_time=22-03-13%2019%3A06%3A55&iridium_latitude=49.2830&iridium_longitude=-123.2476&iridium_cep=3.0&iridium_session_status=0&data=12750a020a0012020a001a020a0022020a002a3f0a3d0a1732322f332f31332d31383a31353a32332e35303030303015530c45421d117ff64225295c0f3e2d3d2a9f4335cdcc74413d0000804040015001580132020a003a020a0042020a004a020a0052020a005a020a0062020a006a020a0072020a00'  # noqa: E501
body2_sensors_ = b'imei=300234068129370&device_type=ROCKBLOCK&serial=16967&momsn=761&transmit_time=22-03-13%2019%3A06%3A55&iridium_latitude=49.2830&iridium_longitude=-123.2476&iridium_cep=3.0&iridium_session_status=0&data=12380a020a0012020a001a020a0022020a002a020a0032020a003a020a0042020a004a020a0052020a005a020a0062020a006a020a0072020a00'  # noqa: E501
f1 = open("sensorBinaryData.txt", "rb")
f2 = open("uccmBinaryData.txt", "rb")

data0 = bytes.fromhex(str(body0_big_uccm).split("data=", 1)[1][:-1])
data1 = bytes.fromhex(str(body1_sensors).split("data=", 1)[1][:-1])
data2 = bytes.fromhex(str(body2_sensors_).split("data=", 1)[1][:-1])
data3 = f1.read()
data4 = f2.read()

data_list = [data0, data1, data2, data3, data4]
for data in data_list:
    print(data)
    try:
        sat.ParseFromString(data)
        if sat.type == Satellite_pb2.Satellite.Type.SENSORS:
            print("Receiving Sensor Data")
            values = helper.sensors_to_root(sat.sensors)
            nt_connection.setValues(values)

        elif sat.type == Satellite_pb2.Satellite.Type.UCCMS:
            print("Receiving UCCM Data")
            values = helper.uccms_to_root(sat.uccms)
            nt_connection.setValues(values)

        else:
            print("Did Not receive Sensor or UCCM data")
    except ...:
        print("Error while parsing or setting values")
print("Done")
f1.close()
f2.close()
