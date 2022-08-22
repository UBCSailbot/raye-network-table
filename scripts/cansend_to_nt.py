from nt_connection.Connection import Connection
from nt_connection.uri import *
import generated_python.Value_pb2 as Value_pb2
import generated_python.Node_pb2 as Node_pb2
import argparse

def main():
    parser = argparse.ArgumentParser(description="Directly send CAN commands to the network table")

    parser.add_argument('-i',
                        '--id',
                        metavar='FRAME_ID',
                        type=str,
                        help="CAN frame ID (ex. 0x100)",
                        required=True)

    parser.add_argument('-d',
                        '--data',
                        metavar="DATA",
                        type=str,
                        help="CAN command data (ex. 0x1234)",
                        required=True)

    args = parser.parse_args()

    nt_connection = Connection()
    nt_connection.Connect()

    value = Value_pb2.Value()
    value.type = Value_pb2.Value.Type.CAN_CMD
    value.can_cmd.can_frame_id = int(args.id, base=0)  # Allow variable bases
    value.can_cmd.can_cmd_data = int(args.data, base=0)
    print(value)

    uri = REMOTE_CAN_CMD
    values = {uri: value}
    nt_connection.setValues(values)

    # node_container = nt_connection.getNodes([uri])
    # node = Node_pb2.Node()
    # node.CopyFrom(node_container[uri])
    # print(node)

if __name__ == "__main__":
    main()