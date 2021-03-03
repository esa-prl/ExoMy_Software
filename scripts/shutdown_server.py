# Author Christos Tranoris
# https://github.com/ctranoris/ExoMy_Software/blob/master/scripts/shutdown_server.py
# 
# Adapted for Python 2
#
# This script runs on raspberry host to safelly shut down the system
# It listens in port 8099. Issue for example http://192.168.2.64:8099/shutdown 
# to shutdown the system. 
# To autostart this script add to crontab -e
# @reboot sudo python /home/pi/ExoMy_Software/scripts/shutdown_server.py

import SocketServer
from BaseHTTPServer import BaseHTTPRequestHandler
from subprocess import call

def some_function():	
	call("sudo shutdown -h now", shell=True)

class MyHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/shutdown':
            # Insert your code here
            some_function()

        self.send_response(200)
        return

print('Shutdown server listening on port 8099...')
httpd = SocketServer.TCPServer(("", 8099), MyHandler)
httpd.serve_forever()