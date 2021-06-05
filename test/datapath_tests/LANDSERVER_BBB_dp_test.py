"""
--- LANDSERVER-ROCKBLOCK-BBB datapath test---

Tests if the waypoint data is sent correctly between the land_satellite_listener and the bbb_satellite_listener.

SSH into the BBB and NUC.
Make sure you run the following jobs:

On the BBB:
~$ ./network_table_server &
~$ ./bbb_satellite_listener 5 5 5 &

On the SERVER:
~$ ./network_table_server &
"""

from nt_connection.Connection import Connection
from nt_connection.Connection import ConnectionError
import generated_python.Value_pb2 as Value_pb2
import generated_python.Satellite_pb2 as Satellite_pb2
import generated_python.Node_pb2 as Node_pb2
import request

"""
Writes a sample waypoint data in the network table.

"""
def write_mock_waypoint_data(nt_connection: Connection, sample_data: int):
    value = Value_pb2.Value()
    value.type = Value_pb2.Value.Type.WAYPOINTS
    gpsCoord = value.waypoints.add()
    gpsCoord.latitude = sample_data
    gpsCoord.longitude = sample_data
    gpsCoord = value.waypoints.add()
    gpsCoord.latitude = sample_data
    gpsCoord.longitude = sample_data

    uri = "waypoints"
    print("Writing mock waypoint:")
    print(value)
    values = {uri: value}
    nt_connection.setValues(values)

"""
Writes mock waypoint data to the network-table on the landserver.

"""
def write_waypoint_to_landserver_nt(land_ssh):
    print("Writing mock waypoint to the landserver")
    try:
        nt_connection = Connection()
        nt_connection.Connect()
    except ConnectionError:
        print("Cannot connect to network-table... Did you run the network table server?")
        pass

    # Write mock waypoint data on the land-server network table
    write_mock_waypoint_data(nt_connection, 5)

    cur_sat = self.init_waypoints()
    uris = ["waypoints"]
    node_container = nt_connection.getNodes(uris)
    node = Node_pb2.Node()
    node.CopyFrom(node_container["waypoints"])
    cur_sat.value.CopyFrom(node.value)
    cur_sat_serial = cur_sat.SerializeToString()
    query = {"imei":"300234068129370","data":cur_sat_serial.hex(),"username":"captain@ubcsailbot.org","password":"raye2020"}
    requests.post(self.ENDPOINT, params=security, data=query)

def test_bbb_satellite_listener(bbb_ssh):
    """
    Verfies the waypoint data sent from the landserver has been correctly transferred to the bbb.
    """

