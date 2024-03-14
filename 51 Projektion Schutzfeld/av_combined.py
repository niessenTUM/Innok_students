# # -*- coding: utf-8 -*-

import rospy
import pygame
import pygame.gfxdraw
from pygame.locals import *
import threading
import time
import sys
import os
import simpleaudio as sa
import RPi.GPIO as GPIO
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String


os.environ['PYGAME_HIDE_SUPPORT_PROMT'] = "hide"

# globale Variable
stop_flag = False
zone = None
rc_activated = False

# initalize pygame
pygame.init()

# Titel des Fensters
pygame.display.set_caption("Visualisierung Schutzfeld")

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

# global vars for determining zone
minimal_distance = None
linear_speed = None
zone = None

stop_flag = False
start_publishing = False

input_linear = [0.0, 0.0, 0.0]
timer_move = [5.0, 60.0, 5.0]

length = len(input_linear)  # Determine number of moves
output = [0] *length        # Prepare vector for output to terminal
for x in range(length):     # Fill vector with commands from above in format LIN{number}, e.g. LIN1.0
    output[x] = f"LIN{input_linear[x]}\n"

# Start node
rospy.init_node('RaspPi', anonymous=False, log_level=rospy.DEBUG) # Initalize ROS-Node

class LaserThread(threading.Thread):
    def __init__(self, iD, name):
        threading.Thread.__init__(self)
        self.iD = iD
        self.name = name

    def blink(self, pins, interval, frequency):
        # Blinken der Laser
        for i in range(frequency):
            # Laser einschalten
            for laser_pin in pins:
                GPIO.output(laser_pin, GPIO.HIGH)
            print("lasers on")
            
            # hold for time
            time.sleep(interval)
            
            # Laser ausschalten
            for laser_pin in pins:
                # Laser ausschalten
                GPIO.output(pins, GPIO.LOW)
            print("lasers off")

            # hold for time
            time.sleep(interval)

    def run(self):
        global zone, stop_flag, rc_activated
        last_played = None

        while not stop_flag:
            # if innok rc allows laser to turn on
            if rc_activated:
                # check zones
                if zone == "safety" and last_played != "safetyzone":

                    # Declare which laser pins should blink
                    laser_pins = [Laser_pin_1, Laser_pin_2, Laser_pin_3]

                    # Blink 3 times with 100ms interval
                    self.blink(laser_pins, 0.1, 3)

                    # Laser einschalten
                    for pin in laser_pins:
                        GPIO.output(pin, GPIO.HIGH)
                    print("lasers on")

                    # raise flag that we were at the safety zone
                    last_played = "safetyzone"

                elif (zone == "warning1" or zone == "warning2") and last_played != "warningzone":

                    # Declare which laser pins should blink
                    laser_pins = [Laser_pin_1, Laser_pin_2, Laser_pin_3]

                    # Blink 3 times with 300ms interval
                    self.blink(laser_pins, 0.5, 3)

                    # Laser einschalten
                    for pin in laser_pins:
                        GPIO.output(pin, GPIO.HIGH)
                    print("lasers on")

                    # raise flag that we were at the warning zone
                    last_played = "warningzone"
                        
                elif zone == "safe" and last_played != "outside":

                    # Declare which laser pins should blink
                    laser_pins = [Laser_pin_1, Laser_pin_2, Laser_pin_3]

                    # Laser einschalten
                    for pin in laser_pins:
                        if not GPIO.input(pin):
                            GPIO.output(pin, GPIO.HIGH)
                    print("lasers on")

                    # raise flag that we were at the warning zone
                    last_played = "outside"

                else:
                    time.sleep(0.1)
            else:
                GPIO.output(Laser_pin_1, GPIO.LOW)
                GPIO.output(Laser_pin_2, GPIO.LOW)
                GPIO.output(Laser_pin_3, GPIO.LOW)

        if stop_flag:
            GPIO.output(Laser_pin_1, GPIO.LOW)
            GPIO.output(Laser_pin_2, GPIO.LOW)
            GPIO.output(Laser_pin_3, GPIO.LOW)

class ProjectorThread(threading.Thread):
    def __init__(self, iD, name):
        threading.Thread.__init__(self)
        self.iD = iD
        self.name = name
        
        self.last_played = None
        
        
    def run(self):
        global zone, stop_flag
        # lokale Variabeln
        # Hintergrundfarbe transparent
        bg_color = (0, 0, 0, 0)
        color1 = pygame.Color(255, 0, 0)  # Rot
        color2 = pygame.Color(255, 255, 0)  # Gelb
        color3 = pygame.Color(0, 255, 0)  # Grün
    
        """Abstand von Lidar Sensor noch berücksichtigen"""
        # parameters of display, projection and robot
        imageHeight = 1080  # Höhe in Pixel des projizierten Bildes
        imageWidth = 1920  # Breite in Pixel des projizierten Bildes
        projectionMeter = 2.0  # Höhe in m der Projektion auf dem Boden (Messung!)
        # projectionDistance = 1.0  # Abstand in m zwischen COG des Roboters und Beginn der Projektion
        pixelPerMeter = imageHeight / projectionMeter  # Verhältnis, um reale Parameter zu bestimmen
        # robotWidth = 0.77  # Breite des Roboters. Vergleichen Sie mit der Bedienungsanleitung/Messung
        
        # prepare screen
        screen = pygame.display.set_mode((imageWidth, imageHeight), pygame.FULLSCREEN | pygame.SRCALPHA)
        screen.fill(bg_color)  # Hintergrundfarbe transparent
        
        center = (imageWidth//2, imageHeight+100)
        # Definieren der Zonenradien
        safety_zone = 1.0
        warning_zone1 = 1.6
        warning_zone2 = 2.0

 
        while not stop_flag:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                    break
                    
                if event.type == KEYDOWN and event.key == K_ESCAPE:
                    pygame.quit()
                    sys.exit()
                    break
                    
            if zone == "safety":
                if self.last_played != "safety":
                    screen.fill(bg_color)  # transparenter Hintergrund
                    pygame.gfxdraw.filled_circle(screen, center[0], center[1], int(warning_zone2 * pixelPerMeter), color2)  # Zeichnen des Kreises der Warningzone 2
                    pygame.gfxdraw.filled_circle(screen, center[0], center[1], int(warning_zone1 * pixelPerMeter), color2)  # Zeichnen des Kreises der Warningzone 1
                    pygame.gfxdraw.filled_circle(screen, center[0], center[1], int(safety_zone * pixelPerMeter), color1)  # Zeichnen des Kreises der Safety-Zone
                    pygame.display.update()
                    
                    time.sleep(0.1)
                    
                    self.last_played = "safety"
            
            elif zone == "warning1" or zone == "warning2":
                if self.last_played != "warning":
                    screen.fill(bg_color)  # transparenter Hintergrund
                    pygame.gfxdraw.filled_circle(screen, center[0], center[1], int(warning_zone2 * pixelPerMeter), color2)  # Zeichnen des Kreises der Warning-Zone 2
                    pygame.gfxdraw.filled_circle(screen, center[0], center[1], int(warning_zone1 * pixelPerMeter), color2)  # Zeichnen des Kreises der Warning-Zone 1
                    pygame.display.update()
                    
                    time.sleep(0.1)
                    
                    self.last_played = "warning"
                    
            elif zone == "warning2":
                if self.last_played != "warning":
                    screen.fill(bg_color)  # transparenter Hintergrund
                    pygame.gfxdraw.filled_circle(screen, center[0], center[1], int(warning_zone2 * pixelPerMeter), color3)  # Zeichnen des Kreises der Warning-Zone 2
                    pygame.display.update()
                    
                    time.sleep(0.1)
                    
                    self.last_played = "warning"
                    
            else:
                screen.fill(bg_color)  # transparenter Hintergrund
                pygame.display.update()
                self.last_played = "outside"

class AcousticThread(threading.Thread):
    def __init__(self, iD, name):
        threading.Thread.__init__(self)
        self.iD = iD 
        self.name = name
    
    def run(self):
        global zone, stop_flag
        last_played = None
        while not stop_flag:
            # if innok rc allows laser to turn on
            if rc_activated:
                # check zones
                if zone == "safety" and last_played != "safetyzone":
                    # Führe zusätzliche Aktorensteuerung durch
                    safetyzone_sound = sa.WaveObject.from_wave_file('/home/ubuntu/Desktop/Communication/safetyzone.wav')
                    play_obj = safetyzone_sound.play()
                    play_obj.wait_done()

                    last_played = "safetyzone"
                
                # Warnraum1
                elif zone == "warning1" and last_played != "warningzone1": 
                    # Führe zusätzliche Aktorensteuerung durch
                    warning1_sound = sa.WaveObject.from_wave_file('/home/ubuntu/Desktop/Communication/warningroom1.wav')
                    play_obj = warning1_sound.play()
                    play_obj.wait_done()
                    
                    last_played = "warningzone1"
                
                # Warnraum2
                elif zone == "warning2" and last_played != "warningzone2":
                    # Führe zusätzliche Aktorensteuerung durch
                    warning2_sound = sa.WaveObject.from_wave_file('/home/ubuntu/Desktop/Communication/Warningroom2.wav')
                    play_obj = warning2_sound.play()
                    play_obj.wait_done()
                    
                    last_played = "warningzone2"
                
                else:
                    # Führe zusätzliche Aktorensteuerung durch
                    while zone == "safe" or zone == None:
                        time.sleep(0.1)
     
class PubSubThread(threading.Thread):
    def __init__(self, iD, name):
        threading.Thread.__init__(self)
        self.iD = iD
        self.name = name
        
        # Create subscribers
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.speed_subscriber = rospy.Subscriber('/odom', Odometry, self.speed_callback)
        self.rc_command_subscriber = rospy.Subscriber('/remote_joy', Joy, self.rc_command_callback)

        # Create publisher
        self.pub_debug = rospy.Publisher('/debug', String, queue_size=1)

    def run(self):
        global stop_flag

        while True:
                        
            if stop_flag:
                self.stop_subscribers()  # Stop the subscribers 
                time.sleep(2)
                pygame.quit()
                # sys.exit()
            
    def scan_callback(self, msg):
        # Extract the ranges and determine the minimum distance
        global minimal_distance
        ranges = msg.ranges
        minimal_distance = min(ranges)
        
    def speed_callback(self, msg):
        # Extract the linear speed
        global linear_speed
        linear_speed = msg.twist.twist.linear.x

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


    def stop_subscribers(self):
        self.scan_subscriber.unregister()
        self.speed_subscriber.unregister()
        self.rc_command_subscriber.unregister()
        
class DriverThread(threading.Thread):
     def __init__(self, iD, name):
         threading.Thread.__init__(self)
         self.iD = iD
         self.name = name
         self.safety_zone = 0.0
         self.warning_zone_1 = 0.0
         self.warning_zone_2 = 0.0   
         self.move_counter = 0  
         self.previous_zone = None      
         
         # Create publisher
         self.pub_debug = rospy.Publisher('/debug', String, queue_size=1)
     
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
        global input_linear, timer_move, length, minimal_distance, linear_speed, zone, stop_flag
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
                        twist.linear.x = 0  # Stop the robot
                        zone = "safety"

                        # Pause the timer 
                        timer_move[self.move_counter] = timer_move[self.move_counter] - (rospy.Time.now() - self.start_time).to_sec()  
                        while minimal_distance is not None and minimal_distance <= self.safety_zone:
                            # Wait until the robot is outside the safety zone
                            rospy.sleep(0.01)
                        self.start_time = rospy.Time.now()  # Update the start time

                    # Check if the minimum distance is within the warning zone 1
                    elif minimal_distance is not None and minimal_distance <= self.warning_zone_1:
                        twist.linear.x = input_linear[self.move_counter] * 0.5  # Reduce speed by half
                        zone = "warning1"
                        
                    # Check if the minimum distance is within the maximum warning zone
                    elif minimal_distance is not None and minimal_distance <= self.warning_zone_2:
                        twist.linear.x = input_linear[self.move_counter]
                        zone = "warning2"
                        
                    else:
                        zone = "safe"

                    # Publish zone if it changed
                    if self.previous_zone != zone:
                        print(f"Zone: {zone}")
                        self.pub_debug.publish(f"Laser | Zone: {zone}")
                        self.previous_zone = zone
                    
                    # Publish velocity message
                    pub.publish(twist)

                self.move_counter += 1

        rospy.sleep(0.01) # Wait until minimal_distance and linear_speed are available
        print("Driver Thread stopped.")
        self.pub_debug.publish("Laser | Driver Thread stopped.")
        stop_flag = True
        print("stop_flag is true")                 
# --------------------------------------------------------------------------
    
# initalize threads
tProjector = ProjectorThread(1, "ProjectorTask")
tAcoustic = AcousticThread(2, "AudioTask")
tLaser = LaserThread(3, "LaserTask")

pub_sub_thread = PubSubThread(4, "RobPubSubThread")
driver = DriverThread(5, "DriverThread")

# start threads
# tProjector.start()
tAcoustic.start()
tLaser.start()

pub_sub_thread.start()
driver.start()

print("Threads started")