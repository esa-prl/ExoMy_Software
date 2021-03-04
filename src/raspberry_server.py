# This script runs on raspberry host to safelly shut down the system
# It listens in port 8099. Issue for example http://192.168.2.64:8001/shutdown 
# to shutdown the system. 
# To autostart this script add to sudo crontab -e
# @reboot sudo python /home/pi/ExoMy_Software/src/raspberry_server.py >> /dev/null 2>&1

import SocketServer
from BaseHTTPServer import BaseHTTPRequestHandler
from subprocess import call

def shutdown():	
	call("sudo shutdown -h now", shell=True)

class web_server(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/shutdown':
            shutdown()

        self.send_response(200)
        return

print('Shutdown server script listening on port 8001...')
httpd = SocketServer.TCPServer(("", 8001), web_server)
httpd.serve_forever()