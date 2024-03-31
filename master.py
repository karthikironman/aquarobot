import RPi.GPIO as GPIO
import time
import sys
import board
import busio
import adafruit_adxl34x
import math

# Initialize GPIO
GPIO.setmode(GPIO.BCM)

# Define motor pins
motor1_pins = [9, 10]
motor2_pins = [5, 8] #changing 11 to 8 to resolve auto rotate problem

# Define gripper pins
pinRotater1 = 19
pinRotater2 = 26
pinGripper1 = 13
pinGripper2 = 6

GPIO_TRIG = 4
GPIO_ECHO = 17

GPIO.setup(GPIO_TRIG, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

# Initialize all GPIO pins
all_pins = motor1_pins + motor2_pins + [pinRotater1, pinRotater2, pinGripper1, pinGripper2]
for pin in all_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

# Initialize PWM for motors
motor1_pwm = GPIO.PWM(motor1_pins[0], 100)  # Frequency = 100Hz
motor2_pwm = GPIO.PWM(motor2_pins[0], 100)

motor1_pwm.start(0)  # Start PWM with duty cycle 0
motor2_pwm.start(0)

# Initialize accelerometer
i2c = busio.I2C(board.SCL, board.SDA)
accelerometer = adafruit_adxl34x.ADXL345(i2c)

# Define angle thresholds
FORWARD_THRESHOLD = 70
BACKWARD_THRESHOLD = -55

# Define distance threshold
DISTANCE_THRESHOLD = 5  # Distance in centimeters

# Define motor control functions
def motor1front():
    motor1_pwm.ChangeDutyCycle(25)  # Set duty cycle
    GPIO.output(motor1_pins[1], GPIO.HIGH)  # Set direction forward
    GPIO.output(motor1_pins[0], GPIO.LOW)  # Start motor

def motor2front():
    motor2_pwm.ChangeDutyCycle(25)  # Set duty cycle
    GPIO.output(motor2_pins[1], GPIO.HIGH)  # Set direction forward
    GPIO.output(motor2_pins[0], GPIO.LOW)  # Start motor

def motor1back():
    motor1_pwm.ChangeDutyCycle(25)  # Set duty cycle
    GPIO.output(motor1_pins[1], GPIO.LOW)  # Set direction backward
    GPIO.output(motor1_pins[0], GPIO.HIGH)  # Start motor

def motor2back():
    motor2_pwm.ChangeDutyCycle(25)  # Set duty cycle
    GPIO.output(motor2_pins[1], GPIO.LOW)  # Set direction backward
    GPIO.output(motor2_pins[0], GPIO.HIGH)  # Start motor

def motor_stop():
    GPIO.output(motor1_pins[0], GPIO.LOW)  # Stop motor1
    GPIO.output(motor2_pins[0], GPIO.LOW)  # Stop motor2
    GPIO.output(motor1_pins[1], GPIO.LOW)  # Stop motor1
    GPIO.output(motor2_pins[1], GPIO.LOW)  # Stop motor2
    motor1_pwm.ChangeDutyCycle(0)
    motor2_pwm.ChangeDutyCycle(0)

# Define gripper control functions
def rotater_forward():
    GPIO.output(pinRotater1, GPIO.HIGH)
    GPIO.output(pinRotater2, GPIO.LOW)
    while True:
        # Read acceleration values
        x, y, z = accelerometer.acceleration

        # Calculate roll angle
        roll = math.atan2(y, math.sqrt(x*x + z*z)) * 180.0 / math.pi

        print("Roll: {:.2f} degrees".format(roll))

        if roll >= FORWARD_THRESHOLD:
            break  # Stop rotation

        time.sleep(0.001)  # Adjust sleep time as needed

    # Stop rotation
    GPIO.output(pinRotater1, GPIO.LOW)
    GPIO.output(pinRotater2, GPIO.LOW)

def rotater_backward():
    GPIO.output(pinRotater1, GPIO.LOW)
    GPIO.output(pinRotater2, GPIO.HIGH)
    while True:
        # Read acceleration values
        x, y, z = accelerometer.acceleration

        # Calculate roll angle
        roll = math.atan2(y, math.sqrt(x*x + z*z)) * 180.0 / math.pi

        print("Roll: {:.2f} degrees".format(roll))

        if roll <= BACKWARD_THRESHOLD:
            break  # Stop rotation

        time.sleep(0.001)  # Adjust sleep time as needed

    # Stop rotation
    GPIO.output(pinRotater1, GPIO.LOW)
    GPIO.output(pinRotater2, GPIO.LOW)

def gripper_close():
    GPIO.output(pinGripper1, GPIO.HIGH)
    GPIO.output(pinGripper2, GPIO.LOW)

def gripper_open():
    GPIO.output(pinGripper1, GPIO.LOW)
    GPIO.output(pinGripper2, GPIO.HIGH)

def gripper_rotate_stop():
    GPIO.output(pinRotater1, GPIO.LOW)
    GPIO.output(pinRotater2, GPIO.LOW)

def gripper_stop():
    GPIO.output(pinGripper1, GPIO.LOW)
    GPIO.output(pinGripper2, GPIO.LOW)

# Function to perform automatic movement based on distance
def automatic_movement():
    while True:
        GPIO.output(GPIO_TRIG, GPIO.LOW)
        time.sleep(0.5)
        GPIO.output(GPIO_TRIG, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIG, GPIO.LOW)

        while GPIO.input(GPIO_ECHO) == 0:
            start_time = time.time()
        while GPIO.input(GPIO_ECHO) == 1:
            bounce_back_time = time.time()

        pulse_duration = bounce_back_time - start_time
        distance = round(pulse_duration * 17150, 2)
        print(f"Distance: {distance} cm")

        if distance > DISTANCE_THRESHOLD:
            motor1front()
            motor2front()
            time.sleep(0.5)  # Adjust speed of movement
            motor_stop()
        else:
            gripper_close()
            time.sleep(3)
            gripper_stop()
            rotater_backward()
            gripper_open()
            time.sleep(3)
            gripper_stop()
            rotater_forward()
            break

try:
    while True:
        user_input = input("Enter command: ")
        if user_input == "quit":
            break
        elif user_input == "front":
            motor1front()
            motor2front()
            time.sleep(0.3)
            motor_stop()
        elif user_input == "back":
            motor1back()
            motor2back()
            time.sleep(0.3)
            motor_stop()
        elif user_input == "rotater_forward":
            rotater_forward()
        elif user_input == "rotater_backward":
            rotater_backward()
        elif user_input == "gripper_tight":
            gripper_close()
            time.sleep(1)
            gripper_stop()
        elif user_input == "gripper_relax":
            gripper_open()
            time.sleep(1)
            gripper_stop()
        elif user_input == "automatic":
            automatic_movement()
        else:
            print("Invalid command")

except KeyboardInterrupt:
    # Clean up GPIO
    GPIO.cleanup()

# Clean up GPIO
GPIO.cleanup()
