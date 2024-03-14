----------------- HOW TO RUN ----------------------------
Run projection_management.py on local PC (Luigi)
- this script runs ssh to the raspPi on Innok and runs the scripts necessary to project lasers and trajectory.

Pinouts:
- Connect the pin 11 (GPIO 17) to the MOSFET Trigger/PWM
- Connect the pin 9 (GND) to the MOSFET GND

---------------------------------------------------------

----------------- FILE EXPLANATION ----------------------
1st configuration (obsolete):

projection_management.py: File on remote PC that calls avcommunication_ros.py via ssh
avcommunication_ros.py: File on the Innok Robot, which calls av_new_ros.py via ssh, and publishes scan and odometry data
av_new_ros.py: File on the RaspPi, which controls the laser, audio, and graphical output
minisend.py: Multi-purpose file to write to and read from Innok or RPi using ssh



2nd configuration:

projection_management.py: File on remote PC that calls av_combined.py on RPi via ssh
av_combined.py = File on RPi that controls laser, audio, grpahical output as well as subscribing LIDAR data and calculating safe zones
----------------------------------------------------------

------------------ Raspberry Pi --------------------------
Operating system: Ubuntu server 20.04.5 LTS (64-bit) from rpi-imager
ROS distro: ROS noetic
GUI: lubuntu 
----------------------------------------------------------
