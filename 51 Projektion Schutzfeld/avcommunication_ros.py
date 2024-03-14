# -*- coding: utf-8 -*-

import rospy
import threading
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import paramiko
import json
import sys
import psutil

# global variables
minimal_distance = None
linear_speed = None
zone_to_send = None

stop_flag = False
start_publishing = False

input_linear = [0.0, 0.0, 0.0]
timer_move = [5.0, 60.0, 5.0]

length = len(input_linear)  # Determine number of moves
output = [0] *length        # Prepare vector for output to terminal
for x in range(length):     # Fill vector with commands from above in format LIN{number}, e.g. LIN1.0
    output[x] = f"LIN{input_linear[x]}\n"

rospy.init_node('Innok_PC', anonymous=True, log_level=rospy.DEBUG) # Initalize ROS-Node

class RobPubSubThread(threading.Thread):
    def __init__(self, iD, name):
        threading.Thread.__init__(self)
        self.iD = iD
        self.name = name
        self.previous_zone = None
        
        # Create subscribers to the topics "/scan" and "/odom"
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.speed_subscriber = rospy.Subscriber('/odom', Odometry, self.speed_callback)
        self.debug_subscriber = rospy.Subscriber('/debug', String, self.debug_callback)

        # Create publisher
        self.pub_laser = rospy.Publisher('cmd_laser', String , queue_size=1)
        self.pub_debug = rospy.Publisher('/debug', String, queue_size=1)

    def run(self):
        global stop_flag, zone_to_send

        while True:
            # print("in loop")
            # if minimal_distance is not None and linear_speed is not None:
                # print('minimal distance: {:.2f}, linear speed: {:.2f}'.format(minimal_distance, linear_speed))
            if zone_to_send is not None:   
                if self.previous_zone != zone_to_send or self.previous_zone is None: 
                    self.previous_zone = zone_to_send
                    if start_publishing:
                        self.pub_laser.publish(zone_to_send)
                        print("Innok | published zone:", zone_to_send)
                        self.pub_debug.publish("Innok | published zone:" + zone_to_send)
                        
            if stop_flag:
                self.stop_subscribers()  # Stop the subscribers 
                self.pub_laser.publish("stop") # Publish stop message to raspberry
                break # exit loop

        sys.exit() # Stop the thread
            
    def scan_callback(self, msg):
        # Extract the ranges and determine the minimum distance
        global minimal_distance
        ranges = msg.ranges
        minimal_distance = min(ranges)
        
    def speed_callback(self, msg):
        # Extract the linear speed
        global linear_speed
        linear_speed = msg.twist.twist.linear.x
    
    def debug_callback(self, msg):
        print(msg.data)

    def stop_subscribers(self):
        self.scan_subscriber.unregister()
        self.speed_subscriber.unregister()

 
        
class DriverThread(threading.Thread):
     def __init__(self, iD, name):
         threading.Thread.__init__(self)
         self.iD = iD
         self.name = name
         self.safety_zone = 0.0
         self.warning_zone_1 = 0.0
         self.warning_zone_2 = 0.0   
         self.move_counter = 0        
         self.current_zone = None
         self.previous_zone = None
         
     def calculate_safety_zone(self, linear_speed):
         # Berechnung der Größe des Schutzfeldes
         if 0.8 < linear_speed <= 1.2:
             stop_distance = 1.1
             safety_margin = 0.2  # 10 cm Sicherheitszuschlag
             safety_zone = stop_distance + safety_margin
             warning_zone_1 = safety_zone + 0.8
             warning_zone_2 = safety_zone + 1.6
         elif linear_speed <= 0.8:
             stop_distance = 0.8
             safety_margin = 0.2  # 10 cm Sicherheitszuschlag
             safety_zone = stop_distance + safety_margin
             warning_zone_1 = safety_zone + 0.6
             warning_zone_2 = safety_zone + 1.2
         elif linear_speed >1.2:    # Code is only meant to work until 1.2 m/s
             stop_distance = 3.0
             safety_margin = 0.2
             safety_zone = stop_distance + safety_margin
             warning_zone_1 = safety_zone + 1.0
             warning_zone_2 = safety_zone + 2.0
         return (safety_zone, warning_zone_1, warning_zone_2)            
     
     def run(self):
        global input_linear, timer_move, length, minimal_distance, linear_speed, zone_to_send, stop_flag
        while self.move_counter < length:
            time.sleep(0.1)
            if minimal_distance is not None and linear_speed is not None:
                # Calculate safety zones
                self.safety_zone, self.warning_zone_1, self.warning_zone_2 = self.calculate_safety_zone(linear_speed)
                # print(self.safety_zone)
                
                pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
                time.sleep(0.01)
                twist = Twist()
                
                # Execute the next move in input_linear
                # Twist message is published for the duration of timer_move[self.move_counter]
                twist.linear.x = input_linear[self.move_counter]
                self.start_time = rospy.Time.now()
                while (rospy.Time.now() - self.start_time) < rospy.Duration(timer_move[self.move_counter]):
                    if minimal_distance is not None and minimal_distance <= self.safety_zone:
                        print("robot stopped")
                        twist.linear.x = 0  # Stop the robot
                        self.current_zone = "safety"
                        if self.previous_zone is None or self.previous_zone != self.current_zone:
                            # Übertrage nur, wenn sich die Zone geändert hat
                            zone_to_send = self.current_zone
                            # print("zone to be sent:", zone_to_send)
                            self.previous_zone = self.current_zone  # Speichere die aktuelle Zone als vorherige Zone
                        timer_move[self.move_counter] = timer_move[self.move_counter] - (rospy.Time.now() - self.start_time).to_sec()  # Pause the timer by subtracting the elapsed time
                        while minimal_distance is not None and minimal_distance <= self.safety_zone:
                            # Wait until the robot is outside the safety zone
                            rospy.sleep(0.01)
                        self.start_time = rospy.Time.now()  # Update the start time
                    # Check if the minimum distance is within the warning zone 1
                    elif minimal_distance is not None and minimal_distance <= self.warning_zone_1:
                        twist.linear.x = input_linear[self.move_counter] * 0.5  # Reduce speed by half
                        self.current_zone = "warning1"
                        if self.previous_zone is None or self.previous_zone != self.current_zone:
                            # Übertrage nur, wenn sich die Zone geändert hat
                            zone_to_send = self.current_zone
                            # print("zone to be sent:", zone_to_send)
                            self.previous_zone = self.current_zone # Speichere die aktuelle Zone als vorherige Zone
                    # Check if the minimum distance is within the maximum warning zone
                    elif minimal_distance is not None and minimal_distance <= self.warning_zone_2:
                        twist.linear.x = input_linear[self.move_counter]
                        self.current_zone = "warning2"
                        if self.previous_zone is None or self.previous_zone != self.current_zone:
                            # Übertrage nur, wenn sich die Zone geändert hat
                            zone_to_send = self.current_zone
                            # print("zone to be sent:", zone_to_send)
                            self.previous_zone = self.current_zone # Speichere die aktuelle Zone als vorherige Zone
                    else:
                        if self.previous_zone == "safety":
                            twist.linear.x = 0  # Stop the robot
                        elif self.previous_zone == "warning1":
                            twist.linear.x = input_linear[self.move_counter] * 0.5
                        elif self.previous_zone == "warning2":
                            twist.linear.x = input_linear[self.move_counter]
                        self.current_zone = "safe"
                        if self.previous_zone is None or self.previous_zone != self.current_zone:
                            # Übertrage nur, wenn sich die Zone geändert hat
                            zone_to_send = self.current_zone
                            # print("zone to be sent:", zone_to_send)
                            self.previous_zone = self.current_zone # Speichere die aktuelle Zone als vorherige Zone
                    pub.publish(twist)
                self.move_counter += 1
        rospy.sleep(0.01) # Wait until minimal_distance and linear_speed are available
        print("Driver Thread stopped.")
        stop_flag = True
        print("stop_flag is true")                 
        
class SignalThread(threading.Thread):
    def __init__(self, iD, name):
        threading.Thread.__init__(self)
        self.iD = iD
        self.name = name

        # Create SSH client
        self.sshRasp =  paramiko.SSHClient()
        self.sshRasp.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.sshRasp.connect("10.21.20.15", port = 22, username='ubuntu', password='heros_ubuntu')

        # # Create publisher
        # self.pub_laser = rospy.Publisher('cmd_laser', String , queue_size=1)

    def run(self):
        global zone_to_send, stop_flag, start_publishing

        # allow ssh to use display by running an X Server, access directory, and run file
        stdin, stdout, stderr = self.sshRasp.exec_command(
            # Setup X Server to show pygame's output
            'export XAUTHORITY=~/.Xauthority\n sleep 0.1\n export DISPLAY=:0.0\n sleep 0.1\n sudo xhost +\n sleep 0.1\n'
            # Setup ROS environment
            'source /opt/ros/noetic/setup.bash\n sleep 0.1\n export ROS_MASTER_URI=http://10.21.20.10:11311\n sleep 0.1\n export ROS_IP=10.21.20.15\n sleep 0.1\n'
            # # Make sure we have proper environment for running the python code
            # 'echo ROS distro: $ROS_DISTRO\n sleep 0.1\n echo $Python environemnt: $PYTHONPATH\n sleep 0.1\n'
            # Run the file
            'cd /home/ubuntu/Desktop/Communication \n python3 av_new_ros.py \n')
        
        # start publishing zones only after ssh to Pi is established
        start_publishing = True

        # while True:
        #     if stop_flag:
        #         print("Signal Thread stopped.")
        #         break
        for line in iter(stdout.readline, ""):      # Save the output that is printed to the console by the program send_data_over_ssh to the variable linearspeed
            print(stdout.readline())
            if stop_flag:
                print("Signal Thread stopped.")
                break

# initalize threads
rob_sub_thread = RobPubSubThread(1, "RobPubSubThread")
driver = DriverThread(2, "DriverThread")
signal = SignalThread(3, "SignalThread")

# start threads
rob_sub_thread.start()
driver.start()
signal.start()

print("Threads started.")


if stop_flag == True:
    #time.sleep(10)  # enough time to deliver stop message to raspberry
    rob_sub_thread.join()
    driver.join()
    signal.join()
    
    sys.exit()