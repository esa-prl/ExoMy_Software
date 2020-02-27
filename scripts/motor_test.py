import Adafruit_PCA9685
import time
import sys
'''
This script helps to test pwm motors with the Adafruit PCA9685 board 
'''

# Set the pin of the motor
pin = int(sys.argv[1])
print('Pin: '+str(pin))


pwm = Adafruit_PCA9685.PCA9685()
# For most motors a pwm frequency of 50Hz is normal
pwm_frequency = 50.0 #Hz
pwm.set_pwm_freq(pwm_frequency)

# The cycle is the inverted frequency converted to milliseconds
cycle = 1.0/pwm_frequency * 1000.0 #ms 

# The time the pwm signal is set to on during the duty cycle
on_time = 2.0 #ms

# Duty cycle is the percentage of a cycle the signal is on
duty_cycle = on_time/cycle

# The PCA 9685 board requests a 12 bit number for the duty_cycle
value = int(duty_cycle*4096.0)

# Ste the motor to the value
pwm.set_pwm(pin, 0,value) 
# Wait for 2 seconds
time.sleep(2.0)
# Turn the motor off
pwm.set_pwm(pin, 0, 0)
