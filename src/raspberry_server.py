# This script runs on raspberry host to for example safely shut down the system
# It listens in port 8001. Issue for example http://192.168.2.64:8001/shutdown 
# to shutdown the system. 
# To autostart this script add to "crontab -e":
# @reboot python /home/pi/ExoMy_Software/src/raspberry_server.py >> /dev/null 2>&1
# Do not use sudo for crontab or python command, as it would run the script as root
# and therefore not access the server gui.

import SocketServer
import os
from BaseHTTPServer import BaseHTTPRequestHandler
from mimetypes import guess_type
from subprocess import call

def shutdown():
    #Shutdown command
	call("sudo shutdown -h now", shell=True)

class web_server(BaseHTTPRequestHandler):
    # ExoMy software and server gui path in home folder of user
    exomy_path = "ExoMy_Software/gui_sv"
    
    def do_GET(self):
        #Shutdown command for example via http://192.168.2.64:8001/shutdown
        if self.path == '/shutdown':
            shutdown()
            self.send_response(200)
            self.end_headers()
        #Else serve normal files and data from the "exomy_path"-server gui folder
        else:
            try:
                if self.path in ("", "/"):
                    #Dummy path
                    filepath = "html/bad.html"
                else:
                    filepath = self.path.lstrip("/")
                    # Define path as current user and then server gui and filepath
                    f = open(os.path.join(os.path.expanduser("~"), self.exomy_path, filepath), "rb")

            except IOError:
                self.send_error(404,'File Not Found: %s ' %  filepath)

            else:
                self.send_response(200)
                #this part handles the mimetypes
                mimetype, _ = guess_type(filepath)
                self.send_header('Content-type', mimetype)
                self.end_headers()
                for s in f:
                    self.wfile.write(s)
            
        return

print('Shutdown server script listening on port 8001...')
httpd = SocketServer.TCPServer(("", 8001), web_server)
httpd.serve_forever()