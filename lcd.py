import smbus
import time
import netifaces
import RPi.GPIO as GPIO

# Define some device parameters
I2C_ADDR = 0x27
LCD_WIDTH = 16

LCD_CHR = 1
LCD_CMD = 0

LCD_LINE_1 = 0x80
LCD_LINE_2 = 0xC0  # Reserved for trash box status

LCD_BACKLIGHT = 0x08

ENABLE = 0b00000100

E_PULSE = 0.0005
E_DELAY = 0.0005

bus = smbus.SMBus(4)

GPIO.setmode(GPIO.BCM)
GPIO.setup(16, GPIO.IN)  # Set up GPIO 16 as input for trash box status

def lcd_init():
    lcd_byte(0x33, LCD_CMD)
    lcd_byte(0x32, LCD_CMD)
    lcd_byte(0x06, LCD_CMD)
    lcd_byte(0x0C, LCD_CMD)
    lcd_byte(0x28, LCD_CMD)
    lcd_byte(0x01, LCD_CMD)
    time.sleep(E_DELAY)

def lcd_byte(bits, mode):
    bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
    bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT

    bus.write_byte(I2C_ADDR, bits_high)
    lcd_toggle_enable(bits_high)

    bus.write_byte(I2C_ADDR, bits_low)
    lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
    time.sleep(E_DELAY)
    bus.write_byte(I2C_ADDR, (bits | ENABLE))
    time.sleep(E_PULSE)
    bus.write_byte(I2C_ADDR, (bits & ~ENABLE))
    time.sleep(E_DELAY)

def lcd_string(message, line):
    message = message.ljust(LCD_WIDTH, " ")
    lcd_byte(line, LCD_CMD)
    for i in range(LCD_WIDTH):
        lcd_byte(ord(message[i]), LCD_CHR)

def get_local_ip():
    try:
        interfaces = netifaces.interfaces()
        for interface in interfaces:
            if interface != 'lo':
                addresses = netifaces.ifaddresses(interface)
                if netifaces.AF_INET in addresses:
                    return addresses[netifaces.AF_INET][0]['addr']
    except:
        pass
    return None

def main():
    lcd_init()
   # GPIO.add_event_detect(16, GPIO.BOTH, callback=trash_status_change, bouncetime=200)  # Add interrupt on GPIO 16

    try:
        while True:
            local_ip = get_local_ip()
            if local_ip:
                lcd_string( local_ip, LCD_LINE_1)
            else:
                lcd_string("No Internet", LCD_LINE_1)

            time.sleep(5)  # Check every 5 seconds for internet connectivity
            trash_status_change()
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        lcd_byte(0x01, LCD_CMD)

def trash_status_change():
    if GPIO.input(16):  # If GPIO 16 is high (trash full)
        lcd_string("Trash Full", LCD_LINE_2)
    else:
        lcd_string("Trash Not Full", LCD_LINE_2)

if __name__ == '__main__':
    main()
