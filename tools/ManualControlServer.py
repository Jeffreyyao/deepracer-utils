import sys, os
sys.path.append("../src")
from MotionControls import MotionControls
from http.server import HTTPServer, BaseHTTPRequestHandler

maxThrottle = 1
maxAngle = -1
throttle = 0
angle = 0

mc = MotionControls()

class ManualControlServer(BaseHTTPRequestHandler):
    def do_GET(self):
        global throttle, angle
        if self.path == "/" or self.path == "/stop":
            angle = 0
            throttle = 0
        elif self.path == "/forward":
            throttle = maxThrottle
        elif self.path == "/backward":
            throttle = -maxThrottle
        elif self.path == "/left":
            angle = -maxAngle
        elif self.path == "/right":
            angle = maxAngle
        elif self.path == "/reset_angle":
            angle = 0
        elif self.path == "/reset_throttle":
            throttle = 0
        elif self.path == "/quit":
            print("shutting down server")
            self.server.socket.close()
            sys.exit()
        print(angle,throttle)
        mc.drive(angle,throttle)
        self.send_response(200)

if __name__=="__main__":
    port = 1234
    server = HTTPServer(("192.168.1.110",port), ManualControlServer)
    print("server started at port "+str(port))
    server.serve_forever()
