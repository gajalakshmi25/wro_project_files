
import RPi.GPIO as GPIO
import time
import cv2

# GPIO setup for motor control
motor_pin_left = 17  # Replace with your left motor GPIO pin
motor_pin_right = 18  # Replace with your right motor GPIO pin
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(motor_pin_left, GPIO.OUT)
GPIO.setup(motor_pin_right, GPIO.OUT)

# Function to control motors
def turn_left():
    GPIO.output(motor_pin_left, GPIO.LOW)
    GPIO.output(motor_pin_right, GPIO.HIGH)
    print("Turning left")

def turn_right():
    GPIO.output(motor_pin_left, GPIO.HIGH)
    GPIO.output(motor_pin_right, GPIO.LOW)
    print("Turning right")

def stop_motors():
    GPIO.output(motor_pin_left, GPIO.LOW)
    GPIO.output(motor_pin_right, GPIO.LOW)
    print("Stopping motors")

# Initialize webcam (USB camera)
camera = cv2.VideoCapture(0,cv2.CAP_V4L2)  # Use 0 for your USB camera
camera.set(3, 640)  # Set width to 640
camera.set(4, 400)  # Set height to 400
time.sleep(2)

# Callback function for trackbars
def nothing(x):
    pass

# Create a window for the trackbars
cv2.namedWindow('Trackbars')

# Create trackbars for adjusting HSV values
cv2.createTrackbar('LH', 'Trackbars', 0, 179, nothing)  # Lower Hue
cv2.createTrackbar('LS', 'Trackbars', 100, 255, nothing)  # Lower Saturation
cv2.createTrackbar('LV', 'Trackbars', 100, 255, nothing)  # Lower Value
cv2.createTrackbar('UH', 'Trackbars', 10, 179, nothing)  # Upper Hue
cv2.createTrackbar('US', 'Trackbars', 255, 255, nothing)  # Upper Saturation
cv2.createTrackbar('UV', 'Trackbars', 255, 255, nothing)  # Upper Value

cv2.createTrackbar('LGH', 'Trackbars', 50, 179, nothing)  # Lower Green Hue
cv2.createTrackbar('LGS', 'Trackbars', 100, 255, nothing)  # Lower Green Saturation
cv2.createTrackbar('LGV', 'Trackbars', 48, 255, nothing)  # Lower Green Value
cv2.createTrackbar('UGH', 'Trackbars', 138, 179, nothing)  # Upper Green Hue
cv2.createTrackbar('UGS', 'Trackbars', 255, 255, nothing)  # Upper Green Saturation
cv2.createTrackbar('UGV', 'Trackbars', 255, 255, nothing)  # Upper Green Value

try:
    while True:
        ret, frame = camera.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Get current positions of all trackbars
        lh = cv2.getTrackbarPos('LH', 'Trackbars')
        ls = cv2.getTrackbarPos('LS', 'Trackbars')
        lv = cv2.getTrackbarPos('LV', 'Trackbars')
        uh = cv2.getTrackbarPos('UH', 'Trackbars')
        us = cv2.getTrackbarPos('US', 'Trackbars')
        uv = cv2.getTrackbarPos('UV', 'Trackbars')

        lgh = cv2.getTrackbarPos('LGH', 'Trackbars')
        lgs = cv2.getTrackbarPos('LGS', 'Trackbars')
        lgv = cv2.getTrackbarPos('LGV', 'Trackbars')
        ugh = cv2.getTrackbarPos('UGH', 'Trackbars')
        ugs = cv2.getTrackbarPos('UGS', 'Trackbars')
        ugv = cv2.getTrackbarPos('UGV', 'Trackbars')

        # Convert the frame to HSV color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define color ranges for red and green using trackbars
        lower_red = (lh, ls, lv)
        upper_red = (uh, us, uv)
        lower_green = (lgh, lgs, lgv)
        upper_green = (ugh, ugs, ugv)

        # Threshold the frame to detect red and green
        mask_red = cv2.inRange(hsv_frame, lower_red, upper_red)
        mask_green = cv2.inRange(hsv_frame, lower_green, upper_green)

        bbox1 = cv2.boundingRect(mask_red)
        bbox2 = cv2.boundingRect(mask_green)

        if bbox1[2] > 0 and bbox1[3] > 0:  # Ensure valid bounding box dimensions
            x, y, w, h = bbox1
            x1, y1, w1, h1 = bbox2

            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.rectangle(frame, (x1, y1), (x1 + w1, y1 + h1), (0, 255, 0), 2)

            cv2.line(frame, (200, 0), (200, 400), (0, 255, 255), 1)
            cv2.line(frame, (440, 0), (440, 400), (0, 255, 255), 1)

            if x1 + int(w1 / 2) > 200 and x1 + int(w1 / 2) < 440:
                cv2.line(frame, (x1 + int(w1 / 2), 0), (x1 + int(w1 / 2), 400), (0, 0, 255), 1)
            else:
                cv2.line(frame, (x1 + int(w1 / 2), 0), (x1 + int(w1 / 2), 400), (0, 255, 0), 1)

            if x + int(w / 2) > 200 and x + int(w / 2) < 440:
                cv2.line(frame, (x + int(w / 2), 0), (x + int(w / 2), 400), (0, 0, 255), 1)
            else:
                cv2.line(frame, (x + int(w / 2), 0), (x + int(w / 2), 400), (0, 255, 0), 1)

        # Check for red and green in the frame
        if cv2.countNonZero(mask_red) > 1000:
            turn_right()
        elif cv2.countNonZero(mask_green) > 1000:
            turn_left()
        else:
            stop_motors()

        # Display the processed frame
        cv2.imshow("Color Detection", frame)

        # Press 'q' to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Clean up GPIO and camera
    GPIO.cleanup()
    camera.release()
    cv2.destroyAllWindows()
