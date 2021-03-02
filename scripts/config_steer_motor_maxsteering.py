import Adafruit_PCA9685
import yaml
import time
import os

config_filename = '../config/exomy.yaml'


def get_steering_motor_pins():
    steering_motor_pins = {}
    with open(config_filename, 'r') as file:
        param_dict = yaml.load(file)

    for param_key, param_value in param_dict.items():
        if('pin_steer_' in str(param_key)):
            steering_motor_pins[param_key] = param_value
    return steering_motor_pins

def get_steering_pwm_neutral_values():
    steering_pwm_neutral_values = {}    
    with open(config_filename, 'r') as file:
        param_dict = yaml.load(file)

    for param_key, param_value in param_dict.items():
        if('steer_pwm_neutral_' in str(param_key)):
            steering_pwm_neutral_values[param_key] = param_value
    return steering_pwm_neutral_values

def get_steering_pwm_range_value():
    steering_pwm_range_value = {}    
    with open(config_filename, 'r') as file:
        param_dict = yaml.load(file)

    for param_key, param_value in param_dict.items():
        if('steer_pwm_range' in str(param_key)):
            steering_pwm_range_value[param_key] = param_value
    return steering_pwm_range_value


def get_position_name(name):
    position_name = ''
    if('_fl' in name):
        position_name = 'Front Left'
    elif('_fr' in name):
        position_name = 'Front Right'
    elif('_cl' in name):
        position_name = 'Center Left'
    elif('_cr' in name):
        position_name = 'Center Right'
    elif('_rl' in name):
        position_name = 'Rear Left'
    elif('_rr' in name):
        position_name = 'Rear Right'

    return position_name

# PWM range around steering neutral position
# steer_pwm_range: 185

def update_config_file(steering_pwm_range_dict):
    output = ''
    with open(config_filename, 'rt') as file:
        for line in file:
            for key, value in steering_pwm_range_dict.items():
                if(key in line):
                    line = line.replace(line.split(': ', 1)[
                                        1], str(value) + '\n')

                    break
            output += line

    with open(config_filename, 'w') as file:
        file.write(output)


if __name__ == "__main__":
    print(
        '''
$$$$$$$$\                     $$\      $$\           
$$  _____|                    $$$\    $$$ |          
$$ |      $$\   $$\  $$$$$$\  $$$$\  $$$$ |$$\   $$\ 
$$$$$\    \$$\ $$  |$$  __$$\ $$\$$\$$ $$ |$$ |  $$ |
$$  __|    \$$$$  / $$ /  $$ |$$ \$$$  $$ |$$ |  $$ |
$$ |       $$  $$<  $$ |  $$ |$$ |\$  /$$ |$$ |  $$ |
$$$$$$$$\ $$  /\$$\ \$$$$$$  |$$ | \_/ $$ |\$$$$$$$ |
\________|\__/  \__| \______/ \__|     \__| \____$$ |
                                           $$\   $$ |
                                           \$$$$$$  |
                                            \______/ 
        '''
    )
    print(
        '''
This script helps you to set the pwm range for the steering motors.
You will iterate over two steering motors and set them to left and right 90 degree position.
Based on their average the other steering motors are configured too.

!! It is recommended to run 'config_steer_motor_neutral.py' first. !!

The determined value is written to the config file.

Commands:
a - Decrease value for current pin
d - Increase value for current pin
q - Finish setting value for current pin

[Every of these commands must be confirmed with the enter key]

ctrl+c - Exit script
------------------------------------------------------------------------------
        '''
    )

    if not os.path.exists(config_filename):
        print("exomy.yaml does not exist. Finish config_motor_pins.py to generate it.")
        exit()

    pwm = Adafruit_PCA9685.PCA9685()
    # For most motors a pwm frequency of 50Hz is normal
    pwm_frequency = 50.0  # Hz
    pwm.set_pwm_freq(pwm_frequency)

    # The cycle is the inverted frequency converted to milliseconds
    cycle = 1.0/pwm_frequency * 1000.0  # ms

    # The time the pwm signal is set to on during the duty cycle
    on_time = 1.5  # ms

    # Duty cycle is the percentage of a cycle the signal is on
    duty_cycle = on_time/cycle

    # The PCA 9685 board requests a 12 bit number for the duty_cycle
    initial_value = int(duty_cycle*4096.0)

    # Get all steering pins
    steering_motor_pins = get_steering_motor_pins()
    pwm_neutral_dict = get_steering_pwm_neutral_values()
    steering_pwm_range_dict = {}
    
    # Reset all steering motors to neutal
    for pin_name, pin_value in steering_motor_pins.items():
        pwm_neutral_name = pin_name.replace('pin_steer_', 'steer_pwm_neutral_')
        pwm_neutral_value = pwm_neutral_dict[pwm_neutral_name]     
        pwm.set_pwm(pin_value, 0, pwm_neutral_value)
    
    # Set Sleep time for better understanding of what is happening
    time.sleep(1)
    
    #Remove 4 steering motors from dict
    for x in range(4):
    	steering_motor_pins.popitem()
    
    # Iterating over selected motors and fine tune the 90 degree left and right value
    for pin_name, pin_value in steering_motor_pins.items():
        pwm_neutral_name = pin_name.replace('pin_steer_', 'steer_pwm_neutral_')
        pwm_range_name = pin_name.replace('pin_steer_', 'steer_pwm_range_')
        pwm_neutral_value = pwm_neutral_dict[pwm_neutral_name] 

        # Set left
        print('####################################') 
        print('Set ' + get_position_name(pin_name) + ' steering motor to 90 degree left position:')
        print('#################################### \n') 
        # Preset a certain angle for faster config
        pwm_left_value = pwm_neutral_value - ( pwm_neutral_value / 2 )
        while(1):
            # Set motor
            pwm.set_pwm(pin_value, 0, pwm_left_value)
            time.sleep(0.1)
            print('Current value: ' + str(pwm_left_value) + '\n')
            input = raw_input(
                'q-set / a-decrease pwm left value/ d-increase pwm left value\n')
            if(input is 'q'):
                print('PWM left value for ' + get_position_name(pin_name) +
                      ' has been set.\n')
                break
            elif(input is 'a'):
                print('Decreased pwm left value')
                pwm_left_value-= 5
            elif(input is 'd'):
                print('Increased pwm left value')
                pwm_left_value += 5
         
        # Set right
        print('####################################')      
        print('Set ' + get_position_name(pin_name) + ' steering motor to 90 degree right position:')
        print('#################################### \n')
        # Preset a certain angle for faster config
        pwm_right_value = pwm_neutral_value + ( pwm_neutral_value / 2 )
        while(1):
            # Set motor
            pwm.set_pwm(pin_value, 0, pwm_right_value)
            time.sleep(0.1)
            print('Current value: ' + str(pwm_right_value) + '\n')
            input = raw_input(
                'q-set / a-decrease pwm right value/ d-increase pwm right value\n')
            if(input is 'q'):
                print('PWM right value for ' + get_position_name(pin_name) +
                      ' has been set.\n')
                break
            elif(input is 'a'):
                print('Decreased pwm right value')
                pwm_right_value-= 5
            elif(input is 'd'):
                print('Increased pwm right value')
                pwm_right_value += 5
        
        pwm.set_pwm(pin_value, 0, pwm_neutral_value)
        
        # Calculate pwm_range_value
        pwm_range_value = abs(pwm_right_value-pwm_left_value) / 2
        #Save pwm_range_value to dict
        steering_pwm_range_dict[pwm_range_name] = pwm_range_value
    
    pwm_range_sum = 0
    pwm_range_count = 0
    
    for pwm_range_name, pwm_range_value in steering_pwm_range_dict.items():
        pwm_range_count = pwm_range_count + 1
        pwm_range_sum = pwm_range_sum + pwm_range_value
    
    pwm_range_average = int(pwm_range_sum / pwm_range_count)
    steering_pwm_range_dict = {"steer_pwm_range": pwm_range_average}

    update_config_file(steering_pwm_range_dict)
    
    print('####################################') 
    print('PWM Range set to: ' + str(pwm_range_average))
    print('Finished configuration!!!')
    print('####################################') 