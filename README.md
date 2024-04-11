# RoboLingo

A project utilizing RaspberryPi4 that controls lasers to show safety zone and a beamer to show current trajectory.

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [Hardware_Data](#hardware_data) 

## Installation

### Safety zone projection setup

1. Connect pin 11 (GPIO 17) on RPi to the MOSFET Trigger/PWM

### Trajectory projection setup

1. Connect the beamer to the HDMI port (@Nicolas bitte befuellen welche Port) on the RPi

## Usage

### Switch button configuration

- To activate only the safety zone projection, toggle the C switch on the Innok Robot RC upwards
- To activate only the trajectory projection, toggle the C switch on the Innok Robot RC downwards
- To activate both projections, toggle the D switch on the Innok Robot RC upwards
- To deactivate both projections, toggle the D switch on the Innok Robot RC downwards

### Running the code

1. Turn the robot on and connect the Luigi PC to the robot's WLAn (tumheros)
2. Check connection to the RPi by opening the terminal on the PC and running "ping ubuntu@10.21.20.15". Wait until you get a respond from the RPi
3. Read the README on the folders "50 Projektion Trajektorie" and "51 Projektion Schutzfeld", make sure the files exist on
the raspberry Pi
4. Run projection_management.py on the Luigi PC
5. After running projection_management.py we need to make sure all python processes are killed. To do this first run    ssh to the Rpi by typing in terminal:
    - ssh ubuntu@10.21.20.15
    - password: heros_ubuntu
6. type "sudo pkill -f python3 -9" and enter

### Debugging

To read debug messages from the code running in the raspberry pi, follow the following steps:
1. ssh to the Innok Robot
2. run rostopic echo /debug

## Hardware_Data

### Raspberry Pi 4 Model B
1. Operating system: Ubuntu server 20.04.5 LTS (64-bit) from rpi-imager
2. ROS distro: ROS noetic
3. GUI: lubuntu (No Screensaver and disable password login)
4. Python version: 3.8.10
5. pip version: 23.3.1
6. The Raspberry Pi has to be connected to Innok's WLAN (tumheros) 

### Remote PC (Luigi PC)
1. Operating system: Ubuntu 20.04.6 LTS 
2. Python version: 3.8.10
3. pip version: 23.3.2