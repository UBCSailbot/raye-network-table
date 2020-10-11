import zmq
from generated_python import Reply_pb2
from generated_python import Request_pb2


class ConnectionError(Exception):
    """Exception class"""

    def __init__(self, message):
        self.message = message
        super().__init__(self.message)


class Connection:
    """Connection to the network table server

    This is the python implementation of Connection.h
    """

    def __init__(self):
        self.context = zmq.Context()
        self.socket = None
        self.connected = False

    def Connect(self):
        """Initiates connection to the network table server

        After this, the other functions in the class can
        be called to get and set data in the network table.
        """
        assert not self.connected

        # Send a request to connect to the network table
        init_socket = self.context.socket(zmq.REQ)
        init_socket.connect('ipc:///tmp/sailbot/NetworkTable')
        init_socket.send(b'connect\x00')

        # Reply will contain location to connect to with ZMQ_PAIR
        reply = init_socket.recv()
        reply_body = 'ipc://'
        reply_body += reply.decode('utf-8')
        self.socket = self.context.socket(zmq.PAIR)
        self.socket.connect(reply_body)
        self.connected = True

    def Disconnect(self):
        """Terminates connection to the network table server

        This allows the server to cleanup files/memory
        associated with this connection.
        """
        assert self.connected
        self.socket.send(b'disconnect\x00')
        self.false = True

    def getNodes(self, uris):
        """Gets the nodes at the specified uris from the network table

        uris - list of strings
        returns - map of nodes
        """
        assert self.connected

        request = Request_pb2.Request()
        request.type = Request_pb2.Request.Type.GETNODES

        for uri in uris:
            request.getnodes_request.uris.append(uri)

        request_body = request.SerializeToString()
        self.socket.send(request_body)
        reply_message = self.socket.recv()

        # Expecting a GetNodesReply containing the nodes
        reply_node = Reply_pb2.Reply()
        reply_node.ParseFromString(reply_message)

        if reply_node.type == Reply_pb2.Reply.Type.ERROR:
            raise ConnectionError(reply_node.error_reply.message_data)

        return reply_node.getnodes_reply.nodes

    def setValues(self, values):
        """Sets values in the network table

        values - maps Strings to a Value_pb2s
        return - None
        """
        assert self.connected

        request = Request_pb2.Request()
        request.type = Request_pb2.Request.Type.SETVALUES

        for key in values:
            # SetValuesRequest containing a uri and a Value
            request.setvalues_request.values[key].CopyFrom(values[key])

        # Expecting an acknowledgement reply
        request_body = request.SerializeToString()
        self.socket.send(request_body)
        reply_message = self.socket.recv()
        reply_node = Reply_pb2.Reply()
        reply_node.ParseFromString(reply_message)

        if reply_node.type == Reply_pb2.Reply.Type.ACK:
            return
        elif reply_node.type == Reply_pb2.Reply.Type.ERROR:
            raise ConnectionError(reply_node.error_reply.message_data)
