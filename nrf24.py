from machine import Pin, SPI
from nrf24l01 import NRF24L01
import time

# Manually configure SPI with correct pins
spi = SPI(0, baudrate=1000000, polarity=0, phase=0,
          sck=Pin(2), mosi=Pin(3), miso=Pin(4))  # Force MISO to GPIO1

# Define Chip Enable (CE) and Chip Select Not (CSN) pins
ce = Pin(1, Pin.OUT)    # GPIO4 as CE
csn = Pin(0, Pin.OUT)   # GPIO0 as CSN

# Initialize NRF24L01
nrf = NRF24L01(spi, csn, ce, payload_size=8)

# Function to send data
def send_data(data):
    nrf.stop_listening()
    try:
        if nrf.send(data):
            print("Data sent successfully")
        else:
            print("Transmission failed")
    except OSError:
        print("Failed to send data")

# Function to receive data
def receive_data():
    nrf.start_listening()
    while True:
        buf = nrf.recv()
        if buf:
            print("Received data:", buf)
        time.sleep(1)

# Example usage
send_data(b'Hello')
receive_data()
