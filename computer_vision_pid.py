import cv2
import numpy as np

print("libraries")

"""Implementation of the PID controller"""
import dataclasses


@dataclasses.dataclass
class ErrorVal:
    """PID error terms"""

    e_sum: float = 0
    d_error: float = 0
    current_error: float = 0
    prev_error: float = 0


class PID:
    """PID controller Class"""

    def _init_(
        self, k_proportaiol: float, k_integral: float, k_derivative: float
    ) -> None:
        """Creates a PID controller using provided PID parameters

        Args:
            k_proportaiol (float): the proportional error constant of the desired PID
            k_integral (float): the Integral error constant of the desired PID
            k_derivative (float): the derivative error constant of the desired PID
        """
        self.k_proportaiol = k_proportaiol
        self.k_integral = k_integral
        self.k_derivative = k_derivative
        self.pid: float = 0
        self.error = ErrorVal()

    def compute(self, ref: float, measured: float) -> float:
        """Computes the PID value based on the given reference and measured output values

        Args:
            ref (float): reference signal that we desire to track
            measured (float): actual measured output of the signal

        Returns:
            float: PID value
        """
        self.error.current_error = ref - measured
        self.error.e_sum += self.error.current_error
        self.error.d_error = self.error.current_error - self.error.prev_error
        self.pid = (
            self.k_proportaiol * self.error.current_error
            + self.k_integral * self.error.e_sum
            + self.k_derivative * self.error.d_error
        )
        self.error.prev_error = self.error.current_error
        return self.pid

    def set_pid(
        self, k_proportaiol: float, k_integral: float, k_derivative: float
    ) -> None:
        """Sets the PID controller constants

        Args:
            k_proportaiol (float): the proportional error constant of the desired PID
            k_integral (float): the Integral error constant of the desired PID
            k_derivative (float): the derivative error constant of the desired PID
        """
        self.k_proportaiol = k_proportaiol
        self.k_integral = k_integral
        self.k_derivative = k_derivative

# Define HSV color range for black line (adjust values if needed)
# You might need to experiment with these values based on your lighting conditions and black line shade
lower_black = np.array([0, 0, 0])
upper_black = np.array([180, 255, 50])  # Adjust upper bound for brightness

# Initialize video capture object
cap = cv2.VideoCapture(0)

# Initialize PID controller
kp = 0.1
ki = 0.0
kd = 0.01
pid = PID(kp, ki, kd)
pid.set_pid(kp, ki, kd)

from gpiozero import Motor, PWMLED

motor1_forward = 4
motor1_backward = 14
enable1_pin = 12

motor2_forward = 18
motor2_backward = 17
enable2_pin = 13

# Initialize motor control pins
motor1 = Motor(motor1_forward, motor1_backward)

motor2 = Motor(motor2_forward, motor2_backward)

enable1 = PWMLED(enable1_pin)
enable2 = PWMLED(enable2_pin)

main_speed = 0.5
old_ret=None
old_frame=None
while True:
    try:
    # Capture frame-by-frame
        ret, frame = cap.read()
    except:
        ret = None
        cap = cv2.VideoCapture(0)
        print('excepttttttttt')
    # Check if frame is captured successfully
    if not ret:
        print("Error capturing frame!")
        frame = old_frame
        ret = old_ret
        cap = cv2.VideoCapture(0)

    old_frame = frame
    old_ret = ret     
    # Convert frame to HSV color space (better for color detection)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask to isolate the black line based on HSV range
    mask = cv2.inRange(hsv, lower_black, upper_black)

    # Apply some noise reduction (optional)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours in the mask (connected white pixels)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find the largest contour (assuming the black line is the biggest object)
    largest_contour = None
    max_area = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > max_area:
            max_area = area
            largest_contour = cnt

    # Draw the largest contour (optional, for visualization)
    if largest_contour is not None:
        cv2.drawContours(frame, [largest_contour], 0, (0, 255, 0), 2)  # Green for detected line
        x, y, w, h = cv2.boundingRect(largest_contour)  # Fixed contour index here
        cv2.line(frame, (x + (w // 2), 200), (x + (w // 2), 250), (255, 0, 0), 3)
        centerx_blk = (x + (w // 2))
        setpoint = 320
        error = setpoint - centerx_blk

        # Compute PID value
        pid_val = pid.compute(setpoint, centerx_blk)

        # Scale PID value to robot motors from 0 to 1 assuming max PID value is 250
        speed = pid_val / 500
        speed = np.clip(speed, -0.5, 0.5)
        #speed = (speed+ 1)/4

        motor1.forward()  # Right turn: reduce speed on left motor
        enable1.value = (main_speed + speed) / 2
        motor2.forward() 
        enable2.value = (main_speed - speed)/2

        # Apply PID value to the robot motors (you need to implement this part)

        centertext = "Error= " + str(error) + "speed= " + str(speed)
        cv2.putText(frame, centertext, (0, 340), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 0), 3)

    # Display the original frame and mask (optional, for debugging)
    cv2.imshow('Mask', mask)
    cv2.imshow('Original Frame', frame)

    # Exit on 'q' key press
    if cv2.waitKey(1) == ord('q'):
        break

# Release capture and close all windows
cap.release()
cv2.destroyAllWindows()