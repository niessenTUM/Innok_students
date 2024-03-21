----------------- FILE EXPLANATION ----------------------
projection_management.py: File on remote PC that calls av_combined.py on RPi via ssh
av_combined.py = File on RPi that controls laser, audio, grpahical output as well as subscribing LIDAR data and calculating safe zones
minisend.py: Multi-purpose file on remote PC to write to and read from Innok or RPi using ssh
----------------------------------------------------------