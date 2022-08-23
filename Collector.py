import RPi.GPIO as GPIO
import time

class sample_col():
    def __init__(self):
        self.pin = 14
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        self.servo = GPIO.PWM(self.pin, 50)
        self.servo.start(0)
    
    def grab_sample(self):
        self.servo.ChangeDutyCycle(2 + 80/18);
        time.sleep(0.5)
        self.servo.ChangeDutyCycle(0);

    def release_sample(self):
        self.servo.ChangeDutyCycle(2 + 35/18);
        time.sleep(0.5)
        self.servo.ChangeDutyCycle(0);

    def close_claws(self):
        self.servo.ChangeDutyCycle(2 + 60/18);
        time.sleep(0.5)
        self.servo.ChangeDutyCycle(0);

    def lift_rock(self):
        self.servo.ChangeDutyCycle(2 + 120/18);
        time.sleep(0.5)
        self.servo.ChangeDutyCycle(0);

    def stop(self):
        self.servo.stop()
        GPIO.cleanup(self.pin)
