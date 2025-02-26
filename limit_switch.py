from machine import Pin

class LimitSwitch:
    def __init__(self, pin):
        self.switch = Pin(pin, Pin.IN, Pin.PULL_UP)

    def is_pressed(self):
        return self.switch.value() == 0
