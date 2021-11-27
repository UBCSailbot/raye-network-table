import zmq
import time
from generated_python import Reply_pb2
from generated_python import Request_pb2
from generated_python import Node_pb2


class ConnectionError(Exception):
    """Exception class"""

    def __init__(self, message):
        self.message = message
        super().__init__(self.message)


class Connection:
    """Connection to the network table server

    This is the python implementation of Connection.h
    """

    def __init__(self, welcome_dir='/tmp/sailbot/'):
        """The default location to connect to is in
        the /tmp/sailbot/ folder. However, for certain tests,
        we change this so that we can run a network table
        without messing with anyone doing development and using
        the default location.
        """
        self.context = zmq.Context()
        self.socket = None
        self.connected = False
        self.welcome_dir = welcome_dir
        self.callbacks = dict()

    def Connect(self):
        """Initiates connection to the network table server

        After this, the other functions in the class can
        be called to get and set data in the network table.
        """
        assert not self.connected

        # Send a request to connect to the network table
        init_socket = self.context.socket(zmq.REQ)
        init_socket.connect('ipc://{}NetworkTable'.format(self.welcome_dir))
        # make sure to null terminate strings
        init_socket.send(b'connect\x00')

        # Reply will contain location to connect to with ZMQ_PAIR
        print("testing")
            
        delaycount = 0
        while not self.connected and delaycount < 5000:
            try:
                reply = init_socket.recv(flags=zmq.NOBLOCK)
            #print(reply.decode('utf-8'))
            
                reply_body = 'ipc://'
                reply_body += reply.decode('utf-8')
                self.socket = self.context.socket(zmq.PAIR)
                self.socket.connect(reply_body)
                self.connected = True
                return
        
            except zmq.Again:
                delaycount += 1
                time.sleep(1)
        
        print("timeout")
        raise ConnectionError("TimeoutError")

    def Disconnect(self):
        """Terminates connection to the network table server

        This allows the server to cleanup files/memory
        associated with this connection.
        """
        assert self.connected
        # make sure to null terminate strings
        self.socket.send(b'disconnect\x00')
        self.connected = False

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

    def Subscribe(self, uri, callback=None):
        """Subscribes to the specified uri in the network table

        uri - a string
        callback - a function that is performed on the given node at the specified uri
        """
        assert self.connected
        request = Request_pb2.Request()
        request.type = Request_pb2.Request.Type.SUBSCRIBE
        request.subscribe_request.uri = uri

        request_body = request.SerializeToString()
        self.socket.send(request_body)

        reply_message = self.socket.recv()
        reply_node = Reply_pb2.Reply()
        reply_node.ParseFromString(reply_message)

        if reply_node.type == Reply_pb2.Reply.Type.ACK:
            self.callbacks[uri] = callback
            return
        elif reply_node.type == Reply_pb2.Reply.Type.ERROR:
            raise ConnectionError(reply_node.error_reply.message_data)

    def manageSocket(self):
        """Polls for any subscribers on the network table and performs its callback function.

        DISCLAIMER: Does not perform the same functionality as the method manageSocket in Connection.cpp.
                    This method only deals with nodes that have subscribed to the network table and performs
                    its corresponding callback function.
        """
        def getReply():
            reply_message = self.socket.recv()
            reply_node = Reply_pb2.Reply()
            reply_node.ParseFromString(reply_message)
            return reply_node

        while True:
            self.socket.poll(-1)
            reply_node = getReply()
            if reply_node.type == Reply_pb2.Reply.Type.SUBSCRIBE:
                uri = reply_node.subscribe_reply.uri
                if self.callbacks[uri] is not None:
                    self.callbacks[uri](reply_node.subscribe_reply.node, uri)
            elif reply_node.type == Reply_pb2.Reply.Type.ERROR:
                raise ConnectionError(reply_node.error_reply.message_data)
