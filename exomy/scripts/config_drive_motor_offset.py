import Adafruit_PCA9685
import yaml
import time

config_filename = '../config/exomy.yaml'


def get_driving_pins():
    pin_list = []
    with open(config_filename, 'r') as file:
        param_dict = yaml.load(file)

    for key, value in param_dict.items():
        if('pin_drive_' in str(key)):
            pin_list.append(value)
    return pin_list


if __name__ == "__main__":

    print(
        '''
This script helps you to set the offset of the driving motors correctly.
It will send the intended signal for "not moving" to all the motors.
On each motor you have to turn the correction screw until the motor really stands still.
        '''
    )

    pwm = Adafruit_PCA9685.PCA9685()
    # For most motors a pwm frequency of 50Hz is normal
    pwm_frequency = 50  # Hz
    pwm.set_pwm_freq(pwm_frequency)

    # The cycle is the inverted frequency converted to milliseconds
    cycle = 1/pwm_frequency * 1000  # ms

    # The time the pwm signal is set to on during the duty cycle
    on_time = 1.5  # ms

    # Duty cycle is the percentage of a cycle the signal is on
    duty_cycle = on_time/cycle

    # The PCA 9685 board requests a 12 bit number for the duty_cycle
    value = int(duty_cycle*4096.0)

    pin_list = get_driving_pins()
    for pin in pin_list:
        pwm.set_pwm(pin, 0, value)
        time.sleep(0.1)

    raw_input('Press any button if you are done to complete configuration\n')

    for pin in pin_list:
        pwm.set_pwm(pin, 0, 0)
