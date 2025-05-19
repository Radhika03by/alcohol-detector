import cv2
import dlib
from scipy.spatial import distance
from smbus2 import SMBus
from RPLCD.i2c import CharLCD
import RPi.GPIO as GPIO
import time

# Setup for I2C LCD display
bus = SMBus(1)
lcd = CharLCD(i2c_expander='PCF8574', address=0x27, port=1, cols=16, rows=2, dotsize=8, charmap='A02', auto_linebreaks=True)

# Setup for buzzer
BUZZER_PIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER_PIN, GPIO.OUT)

# Initialize dlib's face detector and facial landmark predictor
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

# Define indices for eyes landmarks
LEFT_EYE = list(range(36, 42))
RIGHT_EYE = list(range(42, 48))

# Eye aspect ratio (EAR) calculation
def eye_aspect_ratio(eye):
    A = distance.euclidean(eye[1], eye[5])  # Vertical distance
    B = distance.euclidean(eye[2], eye[4])  # Vertical distance
    C = distance.euclidean(eye[0], eye[3])  # Horizontal distance
    ear = (A + B) / (2.0 * C)
    return ear

# Thresholds
EAR_THRESHOLD = 0.25  # EAR below this indicates closed eyes
CONSEC_FRAMES = 20    # Number of consecutive frames to consider as dozing

# Initialize counters
frame_count = 0

# Start capturing video
cap = cv2.VideoCapture(0)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = detector(gray)

        for face in faces:
            landmarks = predictor(gray, face)
            landmarks = [(p.x, p.y) for p in landmarks.parts()]

            left_eye = [landmarks[i] for i in LEFT_EYE]
            right_eye = [landmarks[i] for i in RIGHT_EYE]

            # Compute EAR for both eyes
            left_ear = eye_aspect_ratio(left_eye)
            right_ear = eye_aspect_ratio(right_eye)
            avg_ear = (left_ear + right_ear) / 2.0

            # Calculate eye closure percentage
            eye_closure_percent = max(0, int((1 - avg_ear / EAR_THRESHOLD) * 100))

            # Display percentage on LCD every second (reduce flickering)
            if frame_count % 30 == 0:
                lcd.clear()
                lcd.write_string(f"Eye Close: {eye_closure_percent}%")

            # Check if EAR is below threshold (dozing detection)
            if avg_ear < EAR_THRESHOLD:
                frame_count += 1
                if frame_count >= CONSEC_FRAMES:
                    cv2.putText(frame, "DOZING ALERT!", (50, 100),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
                    GPIO.output(BUZZER_PIN, GPIO.HIGH)  # Turn on buzzer
                    time.sleep(1)
                    GPIO.output(BUZZER_PIN, GPIO.LOW)
                    lcd.clear()
                    lcd.write_string("DOZING ALERT!")
            else:
                frame_count = 0
                GPIO.output(BUZZER_PIN, GPIO.LOW)  # Turn off buzzer

        cv2.imshow("Driver Dozing Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
    lcd.clear()
    GPIO.cleanup()
