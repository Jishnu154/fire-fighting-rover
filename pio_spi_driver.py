# type: ignore

import rp2
from machine import Pin, StateMachine
import time

@rp2.asm_pio(
    sideset_init=rp2.PIO.OUT_LOW,        # SCK initially low
    out_init=rp2.PIO.OUT_LOW,            # MOSI initially low
    autopull=True, pull_thresh=8,        # Automatically pull 8 bits
)
def spi_master():
    wrap_target()
    set(x, 7)                    .side(0)    # Initialize bit counter (7 downto 0)
    label("bit_loop")
    out(pins, 1)                 .side(0) [1]  # Output one bit on MOSI, delay 1
    in_(pins, 1)                 .side(1) [1]  # Sample one bit from MISO while SCK high, delay 1
    nop()                        .side(1) [1]  # Hold SCK high briefly
    nop()                        .side(0) [1]  # Bring SCK low again
    jmp(x_dec, "bit_loop")       .side(0) [1]  # Decrement bit counter and loop
    wrap()

class PIOSPI:
    def __init__(self, sck, mosi, miso, baudrate=1000000):
        self.sm = StateMachine(0, spi_master,
                               freq=baudrate,
                               sideset_base=sck,
                               out_base=mosi,
                               in_base=miso)
        self.sm.active(1)
    
    def write(self, data):
        for b in data:
            self.sm.put(b)
            time.sleep_us(100)  # Adjust delay as needed
