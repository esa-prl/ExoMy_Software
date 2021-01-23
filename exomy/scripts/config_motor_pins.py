import Adafruit_PCA9685
import time
from shutil import copyfile
import os

DRIVE_MOTOR, STEER_MOTOR = [0, 1]

pos_names = {
    1: 'fl',
    2: 'fr',
    3: 'cl',
    4: 'cr',
    5: 'rl',
    6: 'rr',
}

pin_dict = {

}

pwm = Adafruit_PCA9685.PCA9685()
# For most motors a pwm frequency of 50Hz is normal
pwm_frequency = 50.0  # Hz
pwm.set_pwm_freq(pwm_frequency)
# The cycle is the inverted frequency converted to milliseconds
cycle = 1.0/pwm_frequency * 1000.0  # ms

# The time the pwm signal is set to on during the duty cycle
on_time_1 = 2.4  # ms
on_time_2 = 1.5  # ms

# Duty cycle is the percentage of a cycle the signal is on
duty_cycle_1 = on_time_1/cycle
duty_cycle_2 = on_time_2/cycle

# The PCA 9685 board requests a 12 bit number for the duty_cycle
value_1 = 200 #int(duty_cycle_1*4096.0)
value_2 = 400#int(duty_cycle_2*4096.0)

class Motor():
    def __init__(self, pin):

        self.pin_name = 'pin_'
        self.pin_number = pin

    def wiggle_motor(self):

        # Set the motor to the second value
        pwm.set_pwm(self.pin_number, 0, value_2)
        # Wait for 1 seconds
        time.sleep(1.0)
        # Set the motor to the first value
        pwm.set_pwm(self.pin_number, 0, value_1)
        # Wait for 1 seconds
        time.sleep(1.0)
        # Set the motor to neutral
        pwm.set_pwm(self.pin_number, 0, 307)
        # Wait for half seconds
        time.sleep(0.5)
        # Stop the motor
        pwm.set_pwm(self.pin_number, 0, 0)

    def stop_motor(self):
        # Turn the motor off
        pwm.set_pwm(self.pin_number, 0, 0)


def print_exomy_layout():
    print(
        '''
        1 fl-||-fr 2
             ||
        3 cl-||-cr 4
        5 rl====rr 6
        '''
    )


def update_config_file():
    file_name = '../config/exomy.yaml'
    template_file_name = file_name+'.template'

    if not os.path.exists(file_name):
        copyfile(template_file_name, file_name)
        print("exomy.yaml.template was copied to exomy.yaml")

    output = ''
    with open(file_name, 'rt') as file:
        for line in file:
            for key, value in pin_dict.items():
                if(key in line):
                    line = line.replace(line.split(': ', 1)[
                                        1], str(value) + '\n')

                    break
            output += line

    with open(file_name, 'w') as file:
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
###############
Motor Configuration

This scripts leads you through the configuration of the motors.
First we have to find out, to which pin of the PWM board a motor is connected.
Look closely which motor moves and type in the answer.

Ensure to run the script until the end, otherwise your changes will not be saved!
This script can always be stopped with ctrl+c and restarted.
All other controls will be explained in the process.
###############
        '''
    )

    for pin_number in range(16):
        motor = Motor(pin_number)
        motor.stop_motor()

    for pin_number in range(16):
        motor = Motor(pin_number)
        motor.wiggle_motor()
        type_selection = ''
        while(1):
            print("Pin #{}".format(pin_number))
            print(
                'Was it a steering or driving motor that moved, or should I repeat the movement? ')
            type_selection = input('(d)rive (s)teer (r)epeat - (n)one (f)inish_configuration\n')
            if(type_selection == 'd'):
                motor.pin_name += 'drive_'
                print('Good job\n')
                break
            elif(type_selection == 's'):
                motor.pin_name += 'steer_'
                print('Good job\n')
                break                
            elif(type_selection == 'r'):
                print('Look closely\n')
                motor.wiggle_motor()
            elif(type_selection == 'n'):
                print('Skipping pin')
                break
            elif(type_selection == 'f'):
                print('Finishing calibration at pin {}.'.format(pin_number))
                break
            else:
                print('Input must be d, s, r, n or f\n')
        
        if (type_selection == 'd' or type_selection == 's'):
            while(1):
                print_exomy_layout()
                pos_selection = input(
                    'Type the position of the motor that moved.[1-6] or (r)epeat\n')
                if(pos_selection == 'r'):
                    print('Look closely\n')
                else:
                    try:
                        pos = int(pos_selection)
                        if(pos >= 1 and pos <= 6):
                            motor.pin_name += pos_names[pos]
                            break
                        else:
                            print('The input was not a number between 1 and 6\n')
                    except ValueError:
                        print('The input was not a number between 1 and 6\n')
            
            pin_dict[motor.pin_name] = motor.pin_number
            print('Motor set!\n')
            print('########################################################\n')
        elif (type_selection == 'f'):
            break
    

    print('Now we will step through all the motors and check whether they have been assigned correctly.\n')
    print('Press ctrl+c if something is wrong and start the script again. \n')
    
    for pin_name in pin_dict:
        print('moving {}'.format(pin_name))
        print_exomy_layout()        
        
        pin = pin_dict[pin_name]
        motor = Motor(pin)
        motor.wiggle_motor()
        input('Press button to continue')
    
    print("You assigned {}/12 motors.".format(len(pin_dict.keys())))

    print('Write to config file.\n')
    update_config_file()
    print(
    '''
    $$$$$$$$\ $$\           $$\           $$\                       $$\ 
    $$  _____|\__|          \__|          $$ |                      $$ |
    $$ |      $$\ $$$$$$$\  $$\  $$$$$$$\ $$$$$$$\   $$$$$$\   $$$$$$$ |
    $$$$$\    $$ |$$  __$$\ $$ |$$  _____|$$  __$$\ $$  __$$\ $$  __$$ |
    $$  __|   $$ |$$ |  $$ |$$ |\$$$$$$\  $$ |  $$ |$$$$$$$$ |$$ /  $$ |
    $$ |      $$ |$$ |  $$ |$$ | \____$$\ $$ |  $$ |$$   ____|$$ |  $$ |
    $$ |      $$ |$$ |  $$ |$$ |$$$$$$$  |$$ |  $$ |\$$$$$$$\ \$$$$$$$ |
    \__|      \__|\__|  \__|\__|\_______/ \__|  \__| \_______| \_______|
                                                                        
    ''')

