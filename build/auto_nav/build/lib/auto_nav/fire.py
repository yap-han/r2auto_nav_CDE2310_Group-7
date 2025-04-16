#!/usr/bin/env python3

import sys
import time
import signal
import atexit
from gpiozero import Motor, Servo
from time import sleep

MOTOR1_FORWARD_PIN = 1    # Adjust per your wiring
MOTOR1_BACKWARD_PIN = 6
MOTOR2_FORWARD_PIN = 17
MOTOR2_BACKWARD_PIN = 27
SERVO_PIN = 12            # Adjust if needed

# We'll define one function to do the entire firing sequence:
def single_firing_sequence(servo):
    """
    Execute exactly one cycle of your 3-launch servo sequence.
    """
    # Launch #1
    servo.value = -1.0
    sleep(1.1)

    # Interval < 4.0s
    servo.value = 1.0
    sleep(1.0)
    servo.value = 0
    sleep(1.8)

    # Launch #2
    servo.value = -1.0
    sleep(1.1)

    # Interval < 2.0s
    servo.value = 1.0
    sleep(1.1)

    servo.value = 0
    sleep(0.1)

    # Launch #3
    servo.value = -1.0
    sleep(1.1)

    # Return
    servo.value = 1.0
    sleep(1.1)
    servo.value = 0
    sleep(2)


def main():
    # ---------------------------
    # 1) Setup motors
    # ---------------------------
    motor1 = Motor(forward=MOTOR1_FORWARD_PIN, backward=MOTOR1_BACKWARD_PIN)
    motor2 = Motor(forward=MOTOR2_FORWARD_PIN, backward=MOTOR2_BACKWARD_PIN)

    def stop_motors():
        print("[FIRE] Stopping both motors.")
        motor1.stop()
        motor2.stop()

    # Register atexit so motors are always stopped
    atexit.register(stop_motors)

    print("[FIRE] Starting motors (one forward, one backward).")
    # Example: motor1 forward, motor2 backward
    motor1.forward()
    motor2.backward()

    # ---------------------------
    # 2) Setup servo
    # ---------------------------
    servo = Servo(SERVO_PIN)

    def detach_servo():
        print("[FIRE] Detaching servo.")
        servo.detach()

    # Also detach servo on exit
    atexit.register(detach_servo)

    # ---------------------------
    # 3) Catch KeyboardInterrupt
    # ---------------------------
    # We'll wrap the main logic in a try/except so that if user hits Ctrl+C,
    # we stop everything cleanly.
    try:
        print("[FIRE] Executing single firing sequence (three launches).")
        single_firing_sequence(servo)

        print("[FIRE] Firing sequence finished. Now stopping motors.")
        stop_motors()

    except KeyboardInterrupt:
        print("[FIRE] Caught keyboard interrupt!")
        # We'll fall through to the 'finally' block below
        # which triggers atexit cleanup too
        sys.exit(1)

    finally:
        # Ensures no matter what, we shut everything down
        stop_motors()
        detach_servo()
        print("[FIRE] fire.py complete. Exiting now.")


if __name__ == "__main__":
    main()
