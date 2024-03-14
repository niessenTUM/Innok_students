# # -*- coding: utf-8 -*-

import rospy
import pygame
import pygame.gfxdraw
from pygame.locals import *
import threading
import time
import sys
import json
import os
import simpleaudio as sa
import RPi.GPIO as GPIO
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


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

# # Set the ROS environment variables
# rospy.set_param('ROS_MASTER_URI', 'http://10.21.20.10:11311')
# rospy.set_param('ROS_IP', '10.21.10.15')
# rospy.set_param('ROS_HOSTNAME', '10.21.10.15')

# os.environ['ROS_MASTER_URI'] = 'http://10.21.20.10:11311'
# os.environ['ROS_IP'] = '10.21.10.15'
# os.environ['ROS_HOSTNAME'] = '10.21.10.15'

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
                    # Print zone
                    print("zone =", zone)

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
                    # Print zone
                    print("zone =", zone)

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
                    # Print zone
                    print("zone =", zone)

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

    def run(self):
        global zone, stop_flag

        # Declare which laser pins should blink
        laser_pins = [Laser_pin_1, Laser_pin_2, Laser_pin_3]

        while not stop_flag:
            if zone == "safety":
                # Blink with 100ms interval
                self.blink(laser_pins, 0.1, 1)
            elif (zone == "warning1" or zone == "warning2"):
                # Blink with 300ms interval
                self.blink(laser_pins, 0.5, 1)
            elif zone == "safe":
                # Laser einschalten
                for pin in laser_pins:
                    if not GPIO.input(pin):
                        GPIO.output(pin, GPIO.HIGH)
            else:
                # Blink with 1000ms interval
                self.blink(laser_pins, 1, 1)
        
        # # shut down lasers if stop_flag is raised
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

class SignalThread(threading.Thread):
    def __init__(self, iD, name):
        threading.Thread.__init__(self)
        self.iD = iD
        self.name = name
        
        # Subscriber
        self.cmd_laser_subscriber = rospy.Subscriber('/cmd_laser', String, self.cmd_laser_callback)
        self.rc_command_subscriber = rospy.Subscriber('/remote_joy', Joy, self.rc_command_callback)

        # Publisher
        self.debug_publisher = rospy.Publisher('/debug', String, queue_size=1)

        print("init succesful")
        
    def run(self):
        global zone, stop_flag  # wait, time
        while True:
            # print(zone)
            if stop_flag:
                time.sleep(2)
                self.stop_subscribers()
                pygame.quit()
                sys.exit()
                break
            
    def cmd_laser_callback(self,msg):
        global zone, stop_flag
        if(msg.data == "stop"):
            stop_flag = True
        else:
            zone = msg.data

        # log received zone from innok
        self.debug_publisher.publish("RaspPi | Zone updated to: " + msg.data)
        print("RaspPi | Zone updated to:", msg.data)

    def rc_command_callback(self, data):
        global rc_activated
        if data.buttons[4] == 1:
            # print only when a state change occurs
            if not rc_activated:
                self.debug_publisher.publish("RaspPi | RC activated")
            # se flag
            rc_activated = True
        else:
            # print only when a state change occurs
            if rc_activated:
                self.debug_publisher.publish("RaspPi | RC deactivated")
            # reset flag
            rc_activated = False

    def stop_subscribers(self):
        self.cmd_laser_subscriber.unregister()
            
# initalize threads
tProjector = ProjectorThread(1, "ProjectorTask")
tSignal = SignalThread(2, "SignalTask")
tAcoustic = AcousticThread(3, "AudioTask")
tLaser = LaserThread(4, "LaserTask")

# start threads
tProjector.start()
tSignal.start()
tAcoustic.start()
tLaser.start()