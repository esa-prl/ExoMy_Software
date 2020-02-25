import Adafruit_PCA9685
import time

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
        # Set the motor to the second value
        pwm.set_pwm(self.pin_number, 0, value_2)
        # Stop the motor
        pwm.set_pwm(self.pin_number, 0, 0)
        # Wait for 2 seconds

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
First we have to find out to which pin of the PWM board a motor is connected.
Look closely which motor moves and type in the asnwer.

This script can always be stopped with ctrl+c and restarted.
###############
        '''
    )
    for pin_number in range(12):
        motor = Motor(pin_number)
        motor.stop_motor()

    for pin_number in range(12):
        motor = Motor(pin_number)
        motor.wiggle_motor()
        while(1):
            print(
                'Was it a steering or driving motor that moved, or should I repeat the movement? ')
            input = raw_input('(d)rive (s)teer (r)epeat\n')
            if(input == 'd'):
                motor.pin_name += 'drive_'
                print('Good job\n')
                break
            elif(input == 's'):
                motor.pin_name += 'steer_'
                print('Good job\n')
                break
            elif(input == 'r'):
                print('Look closely\n')
                motor.wiggle_motor()
            else:
                print('Input must be d, s or r\n')
        while(1):
            print_exomy_layout()
            input = raw_input(
                'Type the position of the motor that moved.[1-6] or (r)epeat\n')
            if(input == 'r'):
                print('Look closely\n')
                motor.wiggle_motor()
            else:
                try:
                    pos = int(input)
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
    print('All motors set!\n')
    update_config_file()
