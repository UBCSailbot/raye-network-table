'''
Writes sample mock waypoint data to the network-table.
This is used with the BBB_LANDSERVER_dp_test script.

ex usage: python waypoint_data_test [sample longitude value] [sample latitude value]

'''

import sys
from nt_connection.Connection import Connection
from nt_connection.Connection import ConnectionError
import generated_python.Value_pb2 as Value_pb2
import generated_python.Satellite_pb2 as Satellite_pb2
import generated_python.Node_pb2 as Node_pb2


def write_mock_waypoint_data(sample_longitude, sample_latitude):
    """
    Writes a sample waypoint data in the network table.

    """
    try:
        nt_connection = Connection()
        nt_connection.Connect()
    except ConnectionError:
        print("Cannot connect to network-table... Did you run the network table server?")
        pass
    value = Value_pb2.Value()
    value.type = Value_pb2.Value.Type.WAYPOINTS
    gpsCoord = value.waypoints.add()
    gpsCoord.latitude = sample_latitude
    gpsCoord.longitude = sample_longitude
    gpsCoord = value.waypoints.add()
    gpsCoord.latitude = sample_latitude
    gpsCoord.longitude = sample_longitude

    uri = "waypoints"
    print("Writing mock waypoint to the network table: \n")
    print(value)
    values = {uri: value}
    nt_connection.setValues(values)


if __name__ == '__main__':
    sample_longitude = sys.argv[1]
    sample_latitude = sys.argv[2]
    write_mock_waypoint_data(float(sample_longitude), float(sample_latitude))
