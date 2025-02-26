# fire-fighting-rover
# A 2-DOF Robotic Arm and Rover System for Remote Fire Suppression

## Overview
This project focuses on the development of a **remotely controlled robotic rover** equipped with a **2-degree-of-freedom (2-DOF) robotic arm** designed for **fire suppression applications**. The system incorporates a suite of advanced electronic components to ensure reliable operation, including:

- **Raspberry Pi Pico W** serving as the central controller.
- **NRF24L01** wireless transceiver for seamless communication.
- **Three TB6612FNG motor drivers** controlling six DC motors for precise rover movement.
- **DRV8825 stepper motor driver** for stepper motor control.
- **CPLD programmed as a 12-bit shift register** to optimize motor driver management.
- **550 Diaphragm Pump (12V)** integrated with a relay system for controlled water spraying.
- **Dual-axis joystick and trigger switches** facilitating intuitive user control.
- **Limit switches** ensuring safe operation of the robotic arm by preventing overextension.

## Key Features
- **Remote-Controlled Operation:** Wireless joystick-based control utilizing **NRF24L01**.
- **Fire Suppression Mechanism:** Water pump activation via **dedicated joystick switch**.
- **Robotic Arm Precision:** Servo-driven arm ensures targeted fire suppression.
- **Built-in Safety Mechanisms:** Limit switches prevent unintended movement beyond defined boundaries.
- **Custom PCB Integration:** Compact design incorporating essential motor drivers, transceivers, and controllers.

## Hardware Components
| Component | Description |
|-----------|-------------|
| **Microcontroller** | Raspberry Pi Pico W |
| **Wireless Communication** | NRF24L01 Transceiver Module |
| **Motor Drivers** | 3x TB6612FNG |
| **Stepper Motor Driver** | DRV8825 |
| **Motors** | 6x DC motors for rover mobility |
| **Robotic Arm Actuation** | Servo Motor |
| **Water Pump** | 550 Diaphragm Pump (12V) |
| **Power Supply** | 18650 Li-ion Battery |

## Wiring & Connectivity
### **NRF24L01 to Raspberry Pi Pico W**
| NRF24L01 Pin | Raspberry Pi Pico W GPIO |
|-------------|---------------------|
| CSN | GPIO0 |
| MOSI | GPIO3 |
| MISO | GPIO4 |
| SCK | GPIO2 |
| CE | GPIO1 |
| IRQ | GPIO6 |

### **Motor Driver (TB6612FNG) Connections**
| TB6612FNG | Raspberry Pi Pico W GPIO |
|----------|--------------------------|
| PMWA | GPIO16, GPIO18, GPIO20 |
| PMWB | GPIO17, GPIO19, GPIO21 |

### **CPLD to Raspberry Pi Pico W (Shift Register Configuration)**
| CPLD Output | Motor Driver Pins |
|------------|------------------|
| Q0-Q1 | AIN1, AIN2 |
| Q2-Q3 | BIN1, BIN2 |
| Q4-Q5 | AIN1, AIN2 |
| Q6-Q7 | BIN1, BIN2 |
| Q8-Q9 | AIN1, AIN2 |
| Q10-Q11 | BIN1, BIN2 |

## Installation & Setup
### 1. **Install MicroPython on Raspberry Pi Pico W**
- Download the **MicroPython firmware** from [MicroPython.org](https://micropython.org/download/rp2-pico-w/).
- Flash the **UF2 file** to the Raspberry Pi Pico W.

### 2. **Set Up Development Environment**
- Install **VS Code** and the **Pico-Go extension**.
- Clone this repository:
  ```sh
  git clone https://github.com/yourusername/firefighting-rover.git
  cd firefighting-rover
  ```
- Connect the **Pico W** and upload the `main.py` script.

### 3. **Execution & Operation**
- Power on the remote controller and the rover.
- Use the joystick for **rover movement control**.
- Activate the **robotic arm** using the designated trigger switches.
- Engage the **water pump** via the joystick switch for fire suppression.

## Future Enhancements
- **Integration of camera-based fire detection** for semi-autonomous firefighting.
- **Optimized power management solutions** to enhance operational efficiency.
- **Implementation of environmental sensors** (temperature, gas) for advanced fire detection capabilities.

## Contributors
- **Jishnu panicker** - Embedded Systems & PCB Design
- **Abhijith v s** - Mechanical Design & Integration
- **Jesso Johnson,Goutham Sivan** - Software & Communication Protocols

## License
This project is released under the **MIT License**, making it open-source and freely available for modification and distribution.
