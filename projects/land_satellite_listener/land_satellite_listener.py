import Value_pb2
import Satellite_pb2
import Uccms_pb2
import Sensors_pb2
from http.server import HTTPServer, BaseHTTPRequestHandler
import http.server
import sys
import requests
import _thread
import threading
import time
sys.path.append('generated_python/')


class HTTPRequestHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        print("Handling post request")
        content_len = int(self.headers['Content-Length'])
        body = self.rfile.read(content_len)
        sat = Satellite_pb2.Satellite()

        try:
            sat.ParseFromString(body)
            if sat.type == Satellite_pb2.Satellite.Type.SENSORS:
                print("Receiving Sensor Data")
                print(sat.sensors)
            elif sat.type == Satellite_pb2.Satellite.Type.UCCMS:
                print("Receiving UCCM Data")
                print(sat.uccms)
            elif sat.type == Satellite_pb2.Satellite.Type.VALUE:
                print("Receiving Waypoint Data")
                print(sat.value)
            else:
                print("Did Not receive Sensor or UCCM data")
                print(body)

            self.send_response(200)
            self.end_headers()

        except IOError:
            print("Error Decoding incoming data")


class runServer(threading.Thread):
    def __init__(self, port):
        threading.Thread.__init__(self)
        self.port = port

    def run(self):
        httpd = HTTPServer(("", self.port), HTTPRequestHandler)
        httpd.serve_forever()


class runClient(threading.Thread):
    def run(self):
        sat = Satellite_pb2.Satellite()
        sat.type = Satellite_pb2.Satellite.Type.VALUE
        sat.value.type = Value_pb2.Value.Type.WAYPOINTS
        gpsCoord1 = sat.value.waypoints.add()
        gpsCoord1.latitude = 1.1
        gpsCoord1.longitude = 1.1
        gpsCoord1 = sat.value.waypoints.add()
        gpsCoord1.latitude = 1.2
        gpsCoord1.longitude = 1.2
        for x in range(20):
            requests.post(ENDPOINT, data=sat.SerializeToString())
            time.sleep(2)


if len(sys.argv) != 3:
    print("usage: python3 land_receive <server port> <HTTP_POST Endpoint>")

else:
    print("serving at port", sys.argv[1])
    port = int(sys.argv[1])
    ENDPOINT = sys.argv[2]

    server = runServer(port)
    client = runClient()
    server.start()
    client.start()
