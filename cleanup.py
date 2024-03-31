import RPi.GPIO as GPIO

# Set the mode to BCM
GPIO.setmode(GPIO.BCM)

# Define the GPIO pins
pins_to_initialize = [10, 11, 9, 5, 6, 13, 19, 26]

# Initialize each pin to LOW
for pin in pins_to_initialize:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

# Cleanup GPIO
GPIO.cleanup()
