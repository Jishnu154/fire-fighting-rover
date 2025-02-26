from machine import Pin, PWM

class Servo:
    def __init__(self, pin):
        self.pwm = PWM(Pin(pin))
        self.pwm.freq(50)

    def set_angle(self, angle):
        duty = int((angle / 180) * 65535)
        self.pwm.duty_u16(duty)
