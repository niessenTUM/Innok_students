# # -*- coding: utf-8 -*-

import rospy
import threading
import time
import sys
import os
import RPi.GPIO as GPIO
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# global vars
minimal_distance = None
rc_activated = False

os.environ['PYGAME_HIDE_SUPPORT_PROMT'] = "hide"

# GPIO Modus (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Pins für die Laser definieren
Laser_pin_1 = 17  # Pin 11
Laser_pin_2 = 18  # Pin 12
Laser_pin_3 = 20  # Pin 38 at raspberry pi 3 model b

# Richtung der GPIO-Pins festlegen (IN / OUT)
GPIO.setup(Laser_pin_1, GPIO.OUT)
GPIO.setup(Laser_pin_2, GPIO.OUT)
GPIO.setup(Laser_pin_3, GPIO.OUT)

# Initialize laser state to off
GPIO.output(Laser_pin_1, GPIO.LOW)
GPIO.output(Laser_pin_2, GPIO.LOW)
GPIO.output(Laser_pin_3, GPIO.LOW)

# Remove Node if it already exists
os.system("rosnode kill /RaspPi")

# Start node
rospy.init_node('RaspPi', anonymous=False, log_level=rospy.DEBUG) # Initalize ROS-Node

class LaserThread(threading.Thread):
    def __init__(self, iD, name):
        threading.Thread.__init__(self)
        self.iD = iD
        self.name = name
        self.timer = time.time()
        self.blink_interval = 0
        self.safety_zone = 1
        self.warning_zone_1 = 1.5
        self.warning_zone_2 = 2.0   
        self.previous_zone = None

        # Create publisher
        self.pub_debug = rospy.Publisher('/debug', String, queue_size=1) 

    def run(self):
        global rc_activated, minimal_distance
        laser_pins = [Laser_pin_1, Laser_pin_2, Laser_pin_3]

        while True:
            # if innok rc allows laser to turn on
            if rc_activated:

                if self.blink_interval == 0:
                    # Turn on
                    for pin in laser_pins:
                        # if laser not on, turn it on
                        if GPIO.input(pin) == 0:
                            GPIO.output(pin, GPIO.HIGH)
                elif time.time() - self.timer > self.blink_interval:
                    # Blink
                    for pin in laser_pins:
                        GPIO.output(pin, not GPIO.input(pin))

                    # reset timer
                    self.timer = time.time()
                
                if minimal_distance is not None and minimal_distance <= self.safety_zone:
                    if self.previous_zone != "safety":
                        self.blink_interval = 0.5
                        self.pub_debug.publish("Laser | Safety zone")
                        self.previous_zone = "safety"
                elif minimal_distance is not None and minimal_distance <= self.warning_zone_1:
                    if self.previous_zone != "warning1":
                        self.blink_interval = 1
                        self.pub_debug.publish("Laser | Warning zone 1")
                        self.previous_zone = "warning1"
                elif minimal_distance is not None and minimal_distance <= self.warning_zone_2:
                    if self.previous_zone != "warning2":
                        self.blink_interval = 1
                        self.pub_debug.publish("Laser | Warning zone 2")
                        self.previous_zone = "warning2"
                else:
                    if self.previous_zone != "safe":
                        self.blink_interval = 0
                        self.pub_debug.publish("Laser | Safe zone")
                        self.previous_zone = "safe"

            else:
                # Turn of laser
                for pin in laser_pins:
                    GPIO.output(pin, GPIO.LOW)
     
class PubSubThread(threading.Thread):
    def __init__(self, iD, name):
        threading.Thread.__init__(self)
        self.iD = iD
        self.name = name
        
        # Create subscribers
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.rc_command_subscriber = rospy.Subscriber('/remote_joy', Joy, self.rc_command_callback)

        # Create publisher
        self.pub_debug = rospy.Publisher('/debug', String, queue_size=1)

    def run(self):
        rospy.spin()

    def scan_callback(self, msg):
        # Extract the ranges and determine the minimum distance
        global minimal_distance
        ranges = msg.ranges
        minimal_distance = min(ranges)

    def rc_command_callback(self, data):
        global rc_activated

        # if button C is up or D is up
        if data.buttons[28] == 1 or data.buttons[30] == 1:
            # print only when a state change occurs
            if not rc_activated:
                self.pub_debug.publish("Laser | Laser activated")
            rc_activated = True
        # if button C is down or D is down
        if data.buttons[29] == 1 or data.buttons[31] == 1:
            if rc_activated:
                self.pub_debug.publish("Laser | Laser deactivated")
            # reset flag
            rc_activated = False
    
# initalize threads
laser_thread = LaserThread(1, "LaserTask")
pub_sub_thread = PubSubThread(2, "RobPubSubThread")

laser_thread.start()
pub_sub_thread.start()

print("Threads started")