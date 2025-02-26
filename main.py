from machine import Pin, PWM, SPI
import time
import nrf24l01  # Our local library

# Define system states and constants
MOTOR_COUNT = 6
STATE_IDLE = 0
STATE_DRIVING = 1
STATE_ARM_CONTROL = 2
current_state = STATE_IDLE

# NRF24L01 Radio setup
spi = SPI(0, baudrate=1000000, polarity=0, phase=0, 
          sck=Pin(2), mosi=Pin(3), miso=Pin(4))
csn = Pin(0, Pin.OUT, value=1)
ce = Pin(1, Pin.OUT, value=0)
irq = Pin(6, Pin.IN)

# Initialize NRF radio
print("Initializing NRF24L01...")
try:
    nrf = nrf24l01.NRF24L01(spi, csn, ce, payload_size=8)
    nrf.open_tx_pipe(b"\x78\x78\x78\x78\x78")  # Address to transmit to
    nrf.open_rx_pipe(1, b"\x87\x87\x87\x87\x87")  # Address to receive on
    nrf.start_listening()
    print("NRF24L01 initialized successfully")
except Exception as e:
    print("Error initializing NRF24L01:", e)

# Stepper motor setup
stepper_dir = Pin(11, Pin.OUT)
stepper_step = Pin(12, Pin.OUT)

# Servo setup
servo = PWM(Pin(5))
servo.freq(50)  # 50Hz for servo control
servo.duty_u16(4915)  # Middle position (~1.5ms)

# Motor drivers PWM setup
print("Initializing motor drivers...")
pwma1 = PWM(Pin(17))
pwmb1 = PWM(Pin(16))
pwma2 = PWM(Pin(18))
pwmb2 = PWM(Pin(19))
pwma3 = PWM(Pin(20))
pwmb3 = PWM(Pin(21))

motor_pwm = [pwma1, pwmb1, pwma2, pwmb2, pwma3, pwmb3]

# Set PWM frequency for motors
for pwm in motor_pwm:
    pwm.freq(1000)
    pwm.duty_u16(0)  # Start with motors off

# CPLD Shift Register setup
sr_data = Pin(22, Pin.OUT)
sr_storage_clock = Pin(15, Pin.OUT)
sr_clear = Pin(14, Pin.OUT)
sr_shift_clock = Pin(13, Pin.OUT)

# Limit switches
limit_switch1 = Pin(7, Pin.IN, Pin.PULL_UP)
limit_switch2 = Pin(8, Pin.IN, Pin.PULL_UP)

# Initialize the CPLD shift register
def init_shift_register():
    sr_clear.value(0)
    time.sleep_ms(1)
    sr_clear.value(1)
    # Clear all motor directions
    set_motor_directions(0)
    print("Shift register initialized")

# Send 12 bits to the CPLD for motor direction control
def set_motor_directions(directions):
    # directions should be a 12-bit value (0-4095)
    for i in range(11, -1, -1):  # Shift out MSB first
        bit = (directions >> i) & 1
        sr_data.value(bit)
        sr_shift_clock.value(1)
        time.sleep_us(1)  # Small delay for stability
        sr_shift_clock.value(0)
    # Transfer to storage register
    sr_storage_clock.value(1)
    time.sleep_us(1)
    sr_storage_clock.value(0)

# Control DC motors (0-65535 for PWM duty cycle)
def set_motor_speed(motor_num, speed, direction):
    global current_directions
    
    if motor_num < 1 or motor_num > MOTOR_COUNT:
        return
    
    # Map motor numbers to PWM objects
    pwm_index = motor_num - 1
    
    # Set speed via PWM
    motor_pwm[pwm_index].duty_u16(speed)
    
    # Calculate bit positions for this motor in direction register
    base_pos = ((motor_num - 1) // 2) * 4
    if motor_num % 2 == 1:  # Odd numbered motors (1, 3, 5)
        dir_bits = 0b0001 << base_pos if direction else 0b0010 << base_pos
    else:  # Even numbered motors (2, 4, 6)
        dir_bits = 0b0100 << base_pos if direction else 0b1000 << base_pos
    
    # Update direction bits without affecting other motors
    mask = ~(0b1111 << base_pos) & 0xFFF  # Create mask with 0s at bit positions
    current_directions = (current_directions & mask) | dir_bits
    set_motor_directions(current_directions)

# Move stepper motor
def move_stepper(steps, direction, delay_ms=2):
    stepper_dir.value(direction)
    for _ in range(steps):
        if (direction and not limit_switch1.value()) or \
           (not direction and not limit_switch2.value()):
            print("Limit switch activated, stopping stepper")
            break  # Stop if limit switch is activated
            
        stepper_step.value(1)
        time.sleep_ms(delay_ms)
        stepper_step.value(0)
        time.sleep_ms(delay_ms)

# Set servo position (0-180 degrees)
def set_servo(angle):
    # Convert angle to duty cycle
    # 0 degrees = 1ms pulse (~3277 duty)
    # 180 degrees = 2ms pulse (~6554 duty)
    duty = int(3277 + (angle * 3277 / 180))
    servo.duty_u16(duty)

# Map a value from one range to another
def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Process joystick data for rover movement
def process_rover_movement(x, y):
    # Normalize values to -127 to 127 range
    x = x - 127
    y = y - 127
    
    # Apply deadzone
    if abs(x) < 15:
        x = 0
    if abs(y) < 15:
        y = 0
    
    # Calculate differential drive values (tank drive)
    left_speed = y + x
    right_speed = y - x
    
    # Clamp values to valid range
    left_speed = max(min(left_speed, 127), -127)
    right_speed = max(min(right_speed, 127), -127)
    
    # Determine direction and convert to positive values
    left_dir = left_speed >= 0
    right_dir = right_speed >= 0
    left_speed = abs(left_speed)
    right_speed = abs(right_speed)
    
    # Scale to PWM range (0-65535)
    left_speed = int(left_speed * 65535 / 127)
    right_speed = int(right_speed * 65535 / 127)
    
    # Set motor speeds (motors 1-2 for left side, 3-4 for right side)
    set_motor_speed(1, left_speed, left_dir)
    set_motor_speed(2, left_speed, left_dir)
    set_motor_speed(3, right_speed, right_dir)
    set_motor_speed(4, right_speed, right_dir)

# Handle water pump control
def control_water_pump(active):
    if active:
        print("Activating water pump")
        set_motor_speed(5, 65535, True)  # Full power
    else:
        set_motor_speed(5, 0, True)  # Turn off

# Main program
def main():
    global current_directions
    current_directions = 0
    
    # System initialization
    print("Initializing system...")
    init_shift_register()
    
    # Joystick data storage
    joystick = {
        'x1': 127, 'y1': 127,  # Left joystick (centered)
        'x2': 127, 'y2': 127,  # Right joystick (centered)
        'buttons': 0           # Button states
    }
    
    # Previous button state for edge detection
    prev_buttons = 0
    
    # Main control loop
    print("Starting main loop")
    while True:
        # Check for radio data
        if not irq.value():
            try:
                data = nrf.recv()
                if data:
                    # Parse joystick data
                    joystick['x1'] = data[0]
                    joystick['y1'] = data[1]
                    joystick['x2'] = data[2]
                    joystick['y2'] = data[3]
                    joystick['buttons'] = (data[4] << 8) | data[5]
                    
                    # Debug output
                    print(f"Joystick: X1={joystick['x1']} Y1={joystick['y1']} X2={joystick['x2']} Y2={joystick['y2']} Buttons={joystick['buttons']:04x}")
            except Exception as e:
                print("Error receiving data:", e)
        
        # Process rover movement (left joystick)
        process_rover_movement(joystick['x1'], joystick['y1'])
        
        # Process arm control (right joystick)
        if abs(joystick['x2'] - 127) > 20:
            steps = abs(joystick['x2'] - 127) // 20
            direction = joystick['x2'] > 127
            move_stepper(steps, direction)
        
        if abs(joystick['y2'] - 127) > 20:
            angle = int(map_value(joystick['y2'], 0, 255, 0, 180))
            set_servo(angle)
        
        # Process button presses (edge detection)
        buttons = joystick['buttons']
        # Check if pump button just pressed (bit 0)
        if (buttons & 0x01) and not (prev_buttons & 0x01):
            control_water_pump(True)
        elif not (buttons & 0x01) and (prev_buttons & 0x01):
            control_water_pump(False)
        
        prev_buttons = buttons
        
        # Small delay to prevent CPU hogging
        time.sleep_ms(10)

# Run the program
if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("Program error:", e)