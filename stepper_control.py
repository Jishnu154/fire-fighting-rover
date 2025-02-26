from machine import Pin
import time

class StepperMotor:
    def __init__(self, dir_pin, step_pin):
        self.dir = Pin(dir_pin, Pin.OUT)
        self.step = Pin(step_pin, Pin.OUT)

    def step_forward(self, steps=200):
        self.dir.value(1)
        for _ in range(steps):
            self.step.value(1)
            time.sleep_us(1000)
            self.step.value(0)
            time.sleep_us(1000)

    def step_backward(self, steps=200):
        self.dir.value(0)
        for _ in range(steps):
            self.step.value(1)
            time.sleep_us(1000)
            self.step.value(0)
            time.sleep_us(1000)
