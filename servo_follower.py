try:
    import RPi.GPIO as GPIO
except:
    import Mock.GPIO as GPIO

import time


class ServoFollower:
    MAX_PAN = 12.5
    MIN_PAN = 2.5
    MAX_TILT = 12
    MIN_TILT = 7

    def __init__(self, pan_pin: int, tilt_pin: int):
        self.pan_pin = pan_pin
        self.tilt_pin = tilt_pin
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pan_pin, GPIO.OUT)
        GPIO.setup(self.tilt_pin, GPIO.OUT)

        self.pan_servo = GPIO.PWM(self.pan_pin, 50)
        self.tilt_servo = GPIO.PWM(self.tilt_pin, 50)
        self.curr_pan = self.MIN_PAN + (self.MAX_PAN - self.MIN_PAN)/2
        self.curr_tilt = self.MIN_TILT + (self.MAX_TILT - self.MIN_TILT)/2
        self.pan_servo.start(self.curr_pan)
        self.tilt_servo.start(self.curr_tilt)

    def stop(self):
        self.pan_servo.stop()
        self.tilt_servo.stop()

    def get_pan_tilt(self):
        return self.curr_pan, self.curr_tilt

    def set_pan(self, pan: float):
        if pan < 0.0:
            pan = 0.0
        if pan > 1.0:
            pan = 1.0
        self.curr_pan = self.MIN_PAN + (self.MAX_PAN - self.MIN_PAN) * pan
        self.pan_servo.ChangeDutyCycle(self.curr_pan)

    def set_tilt(self, tilt: float):
        if tilt < 0.0:
            tilt = 0.0
        if tilt > 1.0:
            tilt = 1.0

        self.curr_tilt = self.MIN_TILT + (self.MAX_TILT - self.MIN_TILT) * tilt
        self.tilt_servo.ChangeDutyCycle(self.curr_tilt)

    def set_pan_tilt(self, pan: float, tilt: float):
        self.set_pan(pan)
        self.set_tilt(tilt)

    def set_delta_pan_tilt(self, delta_pan: float = 0, delta_tilt: float = 0):
        self.curr_pan += delta_pan
        self.curr_tilt += delta_tilt
        self.set_pan(self.curr_pan)
        self.set_tilt(self.curr_tilt)
