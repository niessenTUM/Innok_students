# RoboLingo

A project utilizing RaspberryPi4 that controls lasers to show safety zone and a beamer to show current trajectory.

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [Hardware_Data](#hardware_data) 

## Installation

### Safety zone projection setup

1. Connect pin 11 (GPIO 17) on RPi to the MOSFET Trigger/PWM
2. Connect the pin 9 (GND) to the MOSFET GND

### Trajectory projection setup

1. Connect the beamer to the HDMI port (@Nicolas bitte befuellen welche Port) on the RPi

## Usage

### Switch button configuration

- To activate only the safety zone projection, toggle the C switch on the Innok Robot RC upwards
- To activate only the trajectory projection, toggle the C switch on the Innok Robot RC downwards
- To activate both projections, toggle the D switch on the Innok Robot RC upwards
- To deactivate both projections, toggle the D switch on the Innok Robot RC downwards

### Running the code

1. Run projection_management.py on the Luigi PC

### Debugging

To read debug messages from the code running in the raspberry pi, follow the following steps:
1. ssh to the Innok Robot
2. run rostopic echo /debug

## Hardware_Data

### Raspberry Pi 4 Model B
1. Operating system: Ubuntu server 20.04.5 LTS (64-bit) from rpi-imager
2. ROS distro: ROS noetic (TBD version number?)
3. GUI: lubuntu (No Screensaver and disable password login)
4. Python version: TBD
5. pip version: TBD
6. The Raspberry Pi has to be connected to Innok's WLAN (tumheros) 