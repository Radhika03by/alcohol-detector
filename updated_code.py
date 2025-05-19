import RPi.GPIO as GPIO
import time
from smbus2 import SMBus
from RPLCD.i2c import CharLCD

# Pin configuration
TRIG = 17
ECHO = 27

MQ3_PIN = 14
BUZZER = 15  # Buzzer pin

# Motor A (Left) pins
ENA = 18  # PWM pin for Motor A
IN1 = 23
IN2 = 24

# Motor B (Right) pins
ENB = 21  # PWM pin for Motor B
IN3 = 20
IN4 = 16

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(MQ3_PIN, GPIO.IN)
GPIO.setup(BUZZER, GPIO.OUT)

GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# Initialize PWM for both motors
pwmA = GPIO.PWM(ENA, 100)  # Motor A PWM at 100 Hz
pwmB = GPIO.PWM(ENB, 100)  # Motor B PWM at 100 Hz
pwmA.start(0)
pwmB.start(0)

# Initialize LCD
bus = SMBus(1)
lcd = CharLCD(i2c_expander='PCF8574', address=0x27, port=1, cols=16, rows=2)

lcd.clear()
lcd.write_string(" Smart Vehicle")
time.sleep(5)

def read_distance():
    """Measures distance using the ultrasonic sensor."""
    GPIO.output(TRIG, False)
    time.sleep(0.05)  # Reduced delay for faster response
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Convert to cm
    return round(distance, 2)

def alcohol_detected():
    """Checks if alcohol is detected."""
    return GPIO.input(MQ3_PIN) == 1

def set_speed(distance):
    """
    Dynamically sets the motor speed based on the distance to an obstacle.
    Closer distance reduces speed; farther distance increases speed.
    """
    if distance > 50:
        speed = 100  # Maximum speed
    elif 20 < distance <= 50:
        speed = int((distance / 50) * 100)  # Scale speed linearly (40% to 100%)
    else:
        speed = 0  # Stop if too close
    pwmA.ChangeDutyCycle(speed)
    pwmB.ChangeDutyCycle(speed)
    return speed

def move_forward(speed):
    """Moves the vehicle forward at the specified speed."""
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def stop_vehicle():
    """Stops the vehicle."""
    pwmA.ChangeDutyCycle(0)
    pwmB.ChangeDutyCycle(0)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

def sound_buzzer(state, duration=0.2):
    """Controls the buzzer sound."""
    if state == "on":
        GPIO.output(BUZZER, GPIO.HIGH)
    elif state == "off":
        GPIO.output(BUZZER, GPIO.LOW)
    elif state == "beep":  # Beep intermittently
        GPIO.output(BUZZER, GPIO.HIGH)
        time.sleep(duration)
        GPIO.output(BUZZER, GPIO.LOW)
        time.sleep(duration)

try:
    while True:
        distance = read_distance()
        
        if alcohol_detected():
            lcd.clear()
            lcd.write_string("Alcohol Detected!")
            sound_buzzer("on")  # Continuous buzzer for alcohol detection
            stop_vehicle()
            time.sleep(1)
            continue

        if distance < 20:  # Stop if obstacle is within 20 cm
            lcd.clear()
            lcd.write_string("Obstacle Ahead!")
            sound_buzzer("beep")  # Beep intermittently for obstacle
            stop_vehicle()
        else:
            speed = set_speed(distance)
            sound_buzzer("off")  # Turn off buzzer when safe
            if speed == 0:
                stop_vehicle()  # Ensure vehicle stops when speed is zero
            else:
                lcd.clear()
                lcd.write_string(f"Dist: {distance}cm")
                lcd.cursor_pos = (1, 0)
                lcd.write_string(f"Speed: {speed}%")
                move_forward(speed)

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Program stopped by user")
    lcd.clear()
    GPIO.cleanup()
