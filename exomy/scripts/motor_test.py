import Adafruit_PCA9685
import time
import sys
'''
This script helps to test pwm motors with the Adafruit PCA9685 board 
Example usage:
python motor_test.py 3

Performs a motor test for the motor connected to pin 3 of the PWM board
'''
# Check if the pin number is given as an argument
if len(sys.argv) < 2:
    print('You must give the pin number of the motor to be tested as argument.')
    print('E.g: python motor_test.py 3')
    print('Tests the motor connected to pin 3.')
    exit()

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

selection = ''

while(selection != '0'):

    print("What do you want to test?")
    print("1. Min to Max oscilation")
    print('2. Incremental positioning')
    print('0. Abort')
    selection = input()

    if (int(selection) == 1):

        min_t = 0.5 # ms
        max_t = 2.5 # ms
        mid_t = min_t+max_t/2

        print("pulsewidth_min = {:.2f}, pulsewidth_max = {:.2f}".format(min_t, max_t))

        # *_dc is the percentage of a cycle the signal is on
        min_dc = min_t/cycle
        max_dc = max_t/cycle
        mid_dc = mid_t/cycle
        
        dc_list = [min_dc, mid_dc, max_dc, mid_dc]
        for dc in dc_list:
            pwm.set_pwm(pin, 0, int(dc*4096.0))
            time.sleep(2.0)
            
    if (int(selection) == 2):

        curr_t = 1.5 # ms
        curr_dc = curr_t/cycle
        
        step_size = 0.1 # ms
        
        step_size_scaling = 0.2
        
        dc_selection = ''
        
        while (dc_selection != '0'):
            dc_selection = raw_input('a-d: change pulsewidth | w-s: change step size | 0: back to menu\n')
            if dc_selection == 'a':
                curr_t = curr_t - step_size
            elif dc_selection == 'd':
                curr_t = curr_t + step_size
            elif dc_selection == 's':
                step_size = step_size*(1-step_size_scaling)
            elif dc_selection == 'w':
                step_size = step_size*(1+step_size_scaling)

            curr_dc = curr_t/cycle
            
            curr_pwm = int(curr_dc*4096.0)
            print("t_current:\t{0:.4f} [ms]\nstep_size:\t{1:.4f} [ms]\ncurr_pwm: {2:.2f}".format(curr_t, step_size, curr_pwm))
                        
            pwm.set_pwm(pin, 0, curr_pwm)
            
        

# The PCA 9685 board requests a 12 bit number for the duty_cycle
pwm.set_pwm(pin, 0, 0)
