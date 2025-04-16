from gpiozero import Servo
from time import sleep

servo = Servo(12)

try:
    while True:
        #1st launch
        servo.value = -1.0
        sleep(1.0)
        
        # interval < 4.0 seconds
        servo.value = 1.0
        sleep(1.0)
        servo.value =0
        sleep(1.7)

        #2nd launch
        servo.value = -1.0
        sleep(0.9)

        #interval < 2.0 seconds
        servo.value = 1.0
        sleep(1.0)

        #3rd launch
        servo.value = -1.0
        sleep(1.0)

        #return to
        servo.value = 1.0
        sleep(1.0)

        servo.value = 0
        sleep(2)

except KeyboardInterrupt:
    print("STOP")
    servo.detach()
