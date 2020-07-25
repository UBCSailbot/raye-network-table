from http.server import HTTPServer, BaseHTTPRequestHandler
import sys
sys.path.append('generated_python/')

import Sensors_pb2
import Uccms_pb2
import Satellite_pb2

class HTTPRequestHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        print("Handling post request")
        content_len = int(self.headers['Content-Length']) 
        body = self.rfile.read(content_len)
        satellite = Satellite_pb2.Satellite()

        try:
            satellite.ParseFromString(body)
            if satellite.type == Satellite_pb2.Satellite.Type.SENSORS:
                print("Receiving Sensor Data")
                print(satellite.sensors)
            elif satellite.type == Satellite_pb2.Satellite.Type.UCCMS:
                print("Receiving UCCM Data")
                print(satellite.uccms)
            else:
                print("Did Not receive Sensor or UCCM data")

        except IOError:
            print("Error Decoding incoming data") 

if len(sys.argv) != 2:
    print("usage: python3 land_receive <server port>")

else:
    print("serving at port", sys.argv[1])
    httpd = HTTPServer(("", int(sys.argv[1])), HTTPRequestHandler)
    httpd.serve_forever()
