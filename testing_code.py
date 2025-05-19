import RPi.GPIO as GPIO
import time
import smbus
import I2C_LCD_driver

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Pin definitions
ALCOHOL_SENSOR_PIN = 26
TRIG_PIN = 17
ECHO_PIN = 27
MOTOR_IN1 = 14
MOTOR_IN2 = 15
MOTOR_IN3 = 20
MOTOR_IN4 = 21
MOTOR_PWM_A = 18  # PWM pin for Motor A
MOTOR_PWM_B = 19  # PWM pin for Motor B

# Motor speed (0-100%)
MOTOR_SPEED_FAST = 75
MOTOR_SPEED_SLOW = 30

# GPIO setup
GPIO.setup(ALCOHOL_SENSOR_PIN, GPIO.IN)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)
GPIO.setup(MOTOR_IN1, GPIO.OUT)
GPIO.setup(MOTOR_IN2, GPIO.OUT)
GPIO.setup(MOTOR_IN3, GPIO.OUT)
GPIO.setup(MOTOR_IN4, GPIO.OUT)
GPIO.setup(MOTOR_PWM_A, GPIO.OUT)
GPIO.setup(MOTOR_PWM_B, GPIO.OUT)

# Set up PWM for motor speed control
pwm_a = GPIO.PWM(MOTOR_PWM_A, 100)  # 100 Hz frequency
pwm_b = GPIO.PWM(MOTOR_PWM_B, 100)
pwm_a.start(0)  # Start with motors off
pwm_b.start(0)

# Initialize LCD display
lcd = I2C_LCD_driver.lcd()

# Function to control motors
def motor_forward(speed):
    GPIO.output(MOTOR_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IN2, GPIO.LOW)
    GPIO.output(MOTOR_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_IN4, GPIO.LOW)
    pwm_a.ChangeDutyCycle(speed)
    pwm_b.ChangeDutyCycle(speed)

def motor_stop():
    GPIO.output(MOTOR_IN1, GPIO.LOW)
    GPIO.output(MOTOR_IN2, GPIO.LOW)
    GPIO.output(MOTOR_IN3, GPIO.LOW)
    GPIO.output(MOTOR_IN4, GPIO.LOW)
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)

# Function to measure distance using the ultrasonic sensor
def get_distance():
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()
    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound (34300 cm/s) / 2
    return round(distance, 2)

try:
    while True:
        # Check alcohol sensor
        if GPIO.input(ALCOHOL_SENSOR_PIN) == GPIO.HIGH:
            print("Alcohol detected! Stopping motors and issuing alert.")
            motor_stop()
            lcd.lcd_display_string("Alcohol Detected!", 1)
            lcd.lcd_display_string("Stopping...", 2)
            time.sleep(1)
            continue

        # Check distance from ultrasonic sensor
        distance = get_distance()
        print(f"Distance: {distance} cm")

        if distance > 40:  # Safe to move forward at full speed
            print("Path is clear. Moving forward at full speed.")
            lcd.lcd_display_string("Path Clear", 1)
            lcd.lcd_display_string("Moving Forward", 2)
            motor_forward(MOTOR_SPEED_FAST)
        
        elif 20 < distance <= 40:  # Slow down when obstacle is near
            print("Obstacle detected. Slowing down.")
            lcd.lcd_display_string("Obstacle Ahead", 1)
            lcd.lcd_display_string("Slowing Down", 2)
            motor_forward(MOTOR_SPEED_SLOW)
        
        else:  # Obstacle very close, stop motors
            print("Obstacle very close! Stopping motors.")
            lcd.lcd_display_string("Obstacle Ahead!", 1)
            lcd.lcd_display_string("Stopping...", 2)
            motor_stop()

        time.sleep(0.5)

except KeyboardInterrupt:
    print("Program stopped by User")

finally:
    motor_stop()
    pwm_a.stop()
    pwm_b.stop()
    lcd.lcd_clear()
    GPIO.cleanup()
