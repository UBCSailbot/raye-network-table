"""
A test script that will poll for network table data to verify the waypoint data sent is correct.
This is used with the BBB_LANDSERVER_dp_test script.

"""

from nt_connection.Connection import Connection
from nt_connection.Connection import ConnectionError
from generated_python import Node_pb2
import argparse


# Subscribe to the function and then print out the results from the network table.
def poll_for_network_table_data(uri):
    def getWaypointData(nt_connection, uri):
        try:
            node_container = nt_connection.getNodes(uri)
            node = Node_pb2.Node()
            node.CopyFrom(node_container[uri[0]])
            latitude = node.value.waypoints[0].latitude
            longitude = node.value.waypoints[0].longitude
            waypoints = [latitude, longitude]
            return waypoints
        except ConnectionError:
            return None

    try:
        nt_connection = Connection()
        nt_connection.Connect()
    except ConnectionError:
        print("Cannot connect to network-table... Did you run the network table server?")
        pass

    prev = getWaypointData(nt_connection, uri)
    current = prev
    while prev == current:
        current = getWaypointData(nt_connection, uri)

    print(current)
    nt_connection.Disconnect()
    return


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="DP test helper script")
    parser.add_argument("-u", "--uri",
                        nargs="*",
                        type=str,
                        help="URI to check from the network-table",
                        default=[])
    args = parser.parse_args()
    uri = args.uri

    poll_for_network_table_data(uri)
