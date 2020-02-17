#!/usr/bin/env python

import rospy
import rospkg
import os
from exomy.msg import Screen
from ST7735 import ST7735
from PIL import Image 

screen = ST7735(
        port=1,
        cs=2,  
        dc=19,
        rst=5,
        backlight=0,               
        rotation=0,
        spi_speed_hz=16000000,
        invert=False,
        width=130,
        height=160,
        offset_left=0,
        offset_top=0,
    )

def screen_callback(message):
    global screen

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('exomy')

    image = Image.new("RGB", (128, 160))
    
    if (message.state == 'happy'):
        image = Image.open(package_path + '/resources/happy_mouth.png')
    elif (message.state == 'surprised'):
        image = Image.open(package_path + '/resources/surprised_mouth.png')
    elif (message.state == 'closed'):
        image = Image.open(package_path + '/resources/closed_mouth.png')
    elif (message.state == 'half_closed'):
        image = Image.open(package_path + '/resources/half_closed_mouth.png')

    image = image.resize((128, 160))
    screen.display(image)    

if __name__ == '__main__':
    rospy.init_node('screen')
    rospy.loginfo("Starting the screen node")

    screen_sub = rospy.Subscriber("/screen", Screen, screen_callback)

    rate = rospy.Rate(10)
    rospy.spin()