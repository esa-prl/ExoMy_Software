import Adafruit_PCA9685
import time
import sys

'''
This script simply stops all the motors, in case they were left in a running state. 
'''
pwm = Adafruit_PCA9685.PCA9685()
# For most motors a pwm frequency of 50Hz is normal
pwm_frequency = 50.0  # Hz
pwm.set_pwm_freq(pwm_frequency)

for pin_number in range(16):
    pwm.set_pwm(pin_number, 0, 0)
