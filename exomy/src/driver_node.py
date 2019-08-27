import rospy
from std_msgs.msg import String

import RPi.GPIO as GPIO
import time
import numpy as np

# Set the GPIO modes
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

# Set variables for the GPIO motor pins
# rr- -rl
# cr- -cl
#    |
# fr- -fl
pin_drive_fr =11 
pin_steer_fr =12 


# Set the GPIO Pin mode to be Output
GPIO.setup(pin_drive_fr, GPIO.OUT)
GPIO.setup(pin_steer_fr, GPIO.OUT)

# PWM characteristics 
pwm_frequency = 50 		# Hz

pwm_low_limit = 2.5#5 	# %
pwm_stop = 7.5 			# %
pwm_upper_limit = 11.5#10	# %

# Set the GPIO to software PWM at 'Frequency' Hertz
pwm_drive_fr = GPIO.PWM(pin_drive_fr, pwm_frequency)
pwm_steer_fr = GPIO.PWM(pin_steer_fr, pwm_frequency)


#for speed in range(FullBack*100, FullForward*100, 25):
#	pwm_drive_fr.start(speed/100)
#	time.sleep(1)

angles = np.linspace(pwm_low_limit, pwm_upper_limit, 5)
pwm_steer_fr.start(pwm_stop)
time.sleep(1)


for angle in angles:
	pwm_steer_fr.ChangeDutyCycle(angle)	
	print(angle)
	time.sleep(1)

GPIO.cleanup()
