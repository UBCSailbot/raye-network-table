"""NetworkTableListener.py: streams data from the network-table to the front-end web application."""

__author__ = "John Ahn (jahn18)"
__copyright__ = "Copyright 2020 UBC Sailbot"


from nt_connection.Connection import ConnectionError
from nt_connection.Connection import Connection
from nt_connection.Help import Help
from nt_connection.uri import *
import generated_python.Value_pb2 as Value_pb2
import generated_python.Node_pb2 as Node_pb2
import requests
import websocket
import json
import random
from datetime import datetime, timezone
import argparse


class NetworkTableListener:
    def __init__(self, ws_connection, back_end_connection):
        # web_socket = websocket.WebSocket()
        # web_socket.connect(ws_connection)
        # self.ws = web_socket
        self.back_end_connection = back_end_connection + "/api/sensors"

    """
    The methods below should be used as a callback function when subscribing to the network table.
    It send all the fields of each sensor in json formatted data to the websocket.
    ex usage.)
    - Create a network table instance
    - Call the method subscribe and input the uri and the callback method for that uri
    - network_table.subscribe('/wind_sensor_1', getWindSensorData)

    """

    def getWindSensorData(self, node, uri):
        wind_sensor_data = json.dumps(
            {
                'sensor_type': 'wind',
                'sensor_id': uri,
                'speed': node.children['iimwv'].children['wind_speed'].value.float_data,
                'angle': node.children['iimwv'].children['wind_angle'].value.int_data,
                'timestamp': str(datetime.utcnow().replace(tzinfo=timezone.utc))
                # 'current': node.children['uccm'].children['current'].value.int_data,
                # 'voltage': node.children['uccm'].children['voltage'].value.int_data,
                # 'temperature': node.children['uccm'].children['temperature'].value.int_data,
                # 'status': node.children['uccm'].children['status'].value.string_data
            }
        )
        print(wind_sensor_data)
        requests.post(self.back_end_connection + "/wind", data=wind_sensor_data)
        # self.ws.send(wind_sensor_data)

    def getWinchMotorData(self, node, uri):
        winch_sensor_data = json.dumps(
            {
                'sensor_type': 'winch_motor',
                'sensor_id': uri,
                'angle': node.children['angle'].value.int_data,
                'timestamp': str(datetime.utcnow().replace(tzinfo=timezone.utc))
                # 'current': node.children['uccm'].children['current'].value.int_data,
                # 'voltage': node.children['uccm'].children['voltage'].value.int_data,
                # 'temperature': node.children['uccm'].children['temperature'].value.int_data,
                # 'status': node.children['uccm'].children['status'].value.string_data
            }
        )
        print(winch_sensor_data)
        requests.post(self.back_end_connection + "/winch_motor", data=winch_sensor_data)
        # self.ws.send(winch_sensor_data)

    def getSailencoderData(self, node, uri):
        sailencoder_sensor_data = json.dumps(
            {
                'sensor_type': 'sailencoder',
                'sensor_id': uri,
                'angle': node.children['boom_angle_data'].children['angle'].value.int_data,
                'timestamp': str(datetime.utcnow().replace(tzinfo=timezone.utc))
                # 'current': node.children['uccm'].children['current'].value.int_data,
                # 'voltage': node.children['uccm'].children['voltage'].value.int_data,
                # 'temperature': node.children['uccm'].children['temperature'].value.int_data,
                # 'status': node.children['uccm'].children['status'].value.string_data
            }
        )
        print(sailencoder_sensor_data)
        requests.post(self.back_end_connection + "/sailencoder", data=sailencoder_sensor_data)
        # self.ws.send(sailencoder_sensor_data)

    def getRudderMotorData(self, node, uri):
        rudder_motor_sensor_data = json.dumps(
            {
                'sensor_type': 'rudder_motor',
                'sensor_id': uri,
                'angle': node.children['angle'].value.float_data,
                'timestamp': str(datetime.utcnow().replace(tzinfo=timezone.utc))
                # 'current': node.children['uccm'].children['current'].value.int_data,
                # 'voltage': node.children['uccm'].children['voltage'].value.int_data,
                # 'temperature': node.children['uccm'].children['temperature'].value.int_data,
                # 'status': node.children['uccm'].children['status'].value.string_data
            }
        )
        print(rudder_motor_sensor_data)
        requests.post(self.back_end_connection + "/rudder_motor", data=rudder_motor_sensor_data)
        # self.ws.send(rudder_motor_sensor_data)

    def getAccelerometerData(self, node, uri):
        accelerometer_sensor_data = json.dumps(
            {
                'sensor_type': 'accelerometer',
                'sensor_id': uri,
                'x_pos': node.children['boat_orientation_data'].children['x_axis_acceleration'].value.float_data,
                'y_pos': node.children['boat_orientation_data'].children['y_axis_acceleration'].value.float_data,
                'z_pos': node.children['boat_orientation_data'].children['z_axis_acceleration'].value.float_data,
                'timestamp': str(datetime.utcnow().replace(tzinfo=timezone.utc))
                # 'current': node.children['uccm'].children['current'].value.int_data,
                # 'voltage': node.children['uccm'].children['voltage'].value.int_data,
                # 'temperature': node.children['uccm'].children['temperature'].value.int_data,
                # 'status': node.children['uccm'].children['status'].value.string_data
            }
        )
        print(accelerometer_sensor_data)
        requests.post(self.back_end_connection + "/accelerometer", data=accelerometer_sensor_data)
        # self.ws.send(accelerometer_sensor_data)

    def getBMSData(self, node, uri):
        bms_sensor_data = json.dumps(
            {
                'sensor_type': 'bms',
                'sensor_id': uri,
                'battery_current': node.children['battery_pack_data'].children['battery_current'].value.float_data,
                'battery_voltage': node.children['battery_pack_data'].children['battery_voltage'].value.float_data,
                'timestamp': str(datetime.utcnow().replace(tzinfo=timezone.utc))
                # 'current': node.children['uccm'].children['current'].value.int_data,
                # 'voltage': node.childrgetWinchMotorDataen['uccm'].children['total_voltage'].value.int_data,
                # 'temperature': node.children['uccm'].children['temperature'].value.int_data,
                # 'status': node.children['uccm'].children['status'].value.string_data
            }
        )
        print(bms_sensor_data)
        requests.post(self.back_end_connection + "/bms", data=bms_sensor_data)
        # self.ws.send(bms_sensor_data)

    def getGPSData(self, node, uri):
        gps_sensor_data = json.dumps(
            {
                'sensor_type': 'gps',
                'sensor_id': uri,
                'timestamp': node.children['gprmc'].children['utc_timestamp'].value.string_data,
                'latitude': node.children['gprmc'].children['latitude'].value.float_data,
                'longitude': node.children['gprmc'].children['longitude'].value.float_data,
                'ground_speed': node.children['gprmc'].children['ground_speed'].value.float_data,
                'true_heading': node.children['gprmc'].children['true_heading'].value.float_data,
                'track_made_good': node.children['gprmc'].children['track_made_good'].value.float_data,
                'magnetic_variation': node.children['gprmc'].children['magnetic_variation'].value.float_data
                # 'current': node.children['uccm'].children['current'].value.int_data,
                # 'voltage': node.children['uccm'].children['voltage'].value.int_data,
                # 'temperature': node.children['uccm'].children['temperature'].value.int_data,
                # 'status': node.children['uccm'].children['status'].value.string_data
            }
        )
        print(gps_sensor_data)
        requests.post(self.back_end_connection + "/gps", gps_sensor_data)
        # self.ws.send(gps_sensor_data)

    def getWaypointData(self, node, uri):
        waypoint_data = json.dumps(
            {
                'sensor_type': 'waypoint',
                'sensor_id': uri,
                'latitude': node.value.waypoints.latitude,
                'longitude': node.value.waypoints.longitude,
                'timestamp': str(datetime.utcnow().replace(tzinfo=timezone.utc))
            }
        )
        print(waypoint_data)
        requests.post(self.back_end_connection + "/waypoint", data=waypoint_data)
        # self.ws.send(waypoint_data)

    def getGyroscopeData(self, node, uri):
        gyroscope_data = json.dumps(
            {
                'sensor_type': 'gyroscope',
                'sensor_id': uri,
                'x_velocity': node.children['angular_motion_data'].children['x_velocity'].value.float_data,
                'y_velocity': node.children['angular_motion_data'].children['y_velocity'].value.float_data,
                'z_velocity': node.children['angular_motion_data'].children['z_velocity'].value.float_data,
                'timestamp': str(datetime.utcnow().replace(tzinfo=timezone.utc))
            }
        )
        print(gyroscope_data)
        requests.post(self.back_end_connection + "/gyroscope", data=gyroscope_data)
        # self.ws.send(gyroscope_data)

    def getAISData(self, node, uri):
        ais_data = json.dumps(
            {
                'sensor_type': 'ais',
                'sensor_id': uri,
                'm_mmsi': node.children['m_mmsi'].value.int_data,
                'm_navigationStatus': node.children['m_navigationStatus'].value.int_data,
                'm_rateOfTurn': node.children['m_rateOfTurn'].value.float_data,
                'm_highAccuracy': node.children['m_highAccuracy'].value.bool_data,
                'm_latitude': node.children['m_latitude'].value.float_data,
                'm_longitude': node.children['m_longitude'].value.float_data,
                'm_sog': node.children['m_sog'].value.float_data,
                'm_cog': node.children['m_cog'].value.float_data,
                'm_trueHeading': node.children['m_trueHeading'].value.int_data,
                'm_timeStamp': node.children['m_timeStamp'].value.int_data,
                'm_maneuverIndicator': node.children['m_maneuverIndicator'].value.int_data,
                'm_timeReceived': node.children['m_timeReceived'].value.int_data,
                'm_rateOfTurnValid': node.children['m_rateOfTurnValid'].value.bool_data,
                'm_sogValid': node.children['m_sogValid'].value.bool_data,
                'm_cogValid': node.children['m_cogValid'].value.bool_data,
                'm_trueHeadingValid': node.children['m_trueHeadingValid'].value.bool_data,
                'm_positionValid': node.children['m_positionValid'].value.bool_data,
                'm_timeStampValid': node.children['m_timeStampValid'].value.bool_data,
                'm_transcieverClass': node.children['m_transcieverClass'].value.int_data,
                'timestamp': str(datetime.utcnow().replace(tzinfo=timezone.utc))
            }
        )
        print(ais_data)
        requests.post(self.back_end_connection + "/ais", data=ais_data)
        # self.ws.send(ais_data)


def main():
    parser = argparse.ArgumentParser(description="Runs on the land server"
                                     "to poll and write to the network table")

    parser.add_argument('-w',
                        '--websocket_server',
                        metavar='WEB_SOCKET_SERVER',
                        type=str,
                        help='server name for the websocket',
                        required=True)

    parser.add_argument('-b',
                        '--backend_server',
                        metavar='BACK_END_SERVER_PORT',
                        type=str,
                        help='server name for the backend server',
                        required=True)

    args = parser.parse_args()

    network_table = NetworkTableListener(args.websocket_server, args.backend_server)
    nt_connection = Connection()
    print("Connecting to Network Table...")
    nt_connection.Connect()

    # Subscribe to sailencoder
    nt_connection.Subscribe(SAILENCODER, network_table.getSailencoderData)
    # Subscribe to wind sensors
    nt_connection.Subscribe(WIND1, network_table.getWindSensorData)
    nt_connection.Subscribe(WIND2, network_table.getWindSensorData)
    nt_connection.Subscribe(WIND3, network_table.getWindSensorData)
    # Subscribe to bms
    nt_connection.Subscribe(BMS1, network_table.getBMSData)
    nt_connection.Subscribe(BMS2, network_table.getBMSData)
    nt_connection.Subscribe(BMS3, network_table.getBMSData)
    nt_connection.Subscribe(BMS4, network_table.getBMSData)
    nt_connection.Subscribe(BMS5, network_table.getBMSData)
    nt_connection.Subscribe(BMS6, network_table.getBMSData)
    # Subscribe to gps
    nt_connection.Subscribe(GPS_CAN, network_table.getGPSData)
    nt_connection.Subscribe(GPS_AIS, network_table.getGPSData)
    # Subscribe to accelerometer
    nt_connection.Subscribe(ACCELEROMETER, network_table.getAccelerometerData)
    # Subscribe to gyroscope
    nt_connection.Subscribe(GYROSCOPE, network_table.getGyroscopeData)
    # Subscribe to rudder
    nt_connection.Subscribe(RUDDER_PORT, network_table.getRudderMotorData)
    nt_connection.Subscribe(RUDDER_STBD, network_table.getRudderMotorData)
    # Subscribe to winch
    nt_connection.Subscribe(WINCH_MAIN, network_table.getWinchMotorData)
    nt_connection.Subscribe(WINCH_JIB, network_table.getWinchMotorData)
    # Subscribe to waypoints
    nt_connection.Subscribe(WAYPOINTS_GP, network_table.getWaypointData)

    # Polls the data from the network table
    print("Polling data from Network Table...")
    nt_connection.manageSocket()


if __name__ == '__main__':
    main()
