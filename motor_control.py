from machine import Pin, PWM

class MotorController:
    def __init__(self, pin1, pin2):
        self.pwm1 = PWM(Pin(pin1))
        self.pwm2 = PWM(Pin(pin2))
        self.pwm1.freq(1000)
        self.pwm2.freq(1000)

    def forward(self):
        self.pwm1.duty_u16(50000)
        self.pwm2.duty_u16(0)

    def backward(self):
        self.pwm1.duty_u16(0)
        self.pwm2.duty_u16(50000)

    def stop(self):
        self.pwm1.duty_u16(0)
        self.pwm2.duty_u16(0)
