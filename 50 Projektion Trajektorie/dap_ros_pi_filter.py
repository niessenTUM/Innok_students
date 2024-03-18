import threading
import os
import pygame
import sys
import math
import numpy as np
from scipy.signal import butter, filtfilt
import rospy
import time
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

#global variables
switch = True #is switch in correct position (in the middle)
linear = 0 #current linear velocity
rotation = 0 #current rotational velocity

#initalize pygame
pygame.init()

#title of window
pygame.display.set_caption("Visualisierung Drehwinkel")

# Initalize ROS-Node
rospy.init_node('RaspCi', anonymous=False, log_level=rospy.DEBUG)




#function to calculate new driving angle
def butter_lowpass_filter(data, cutoff, fs, order):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    # Get the filter coefficients
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y

#class of display
class DisplayThread(threading.Thread):
    def __init__(self, iD, name):
        threading.Thread.__init__(self)
        self.iD = iD
        self.name = name

        # ####Local Variables
        self.deg_robot_old = 90
        self.lin_old = 0
        self.rot_old = 0

        self.k_old = 0 # old state of printing

    def run(self):

        #####Global Variables used
        global linear, rotation, switch

        #####Constants
        imageHeight = 1080  # Height in pixels of projected image
        imageWidth = 1920  # Width in pixel of projected image
        projectionMeter = 2.465  # Height in m of projection on floor (measure!)
        projectionDistance = 1.415  # Distance in m between Robot's COG and beginning of projection
        pixelPerMeter = imageHeight / projectionMeter  # Ratio to determine real parameters
        robotWidth = 0.76  # Robot's width. Compare with manual / measure
        middleAxis = pygame.math.Vector2(imageWidth / 2, imageHeight + projectionDistance * pixelPerMeter) # spot on projection display of the middle of the rotation axis
        rectArea = [(imageWidth - robotWidth * pixelPerMeter) / 2, 0, robotWidth * pixelPerMeter, imageHeight] # rectangle for projection of driving forward
        angledifmax = 1 # degree range (deviation from 90 degrees (=forward)) where rectangle instead of radius is projected
        deg_robot_array = np.ones(30)*90 # stored data of the last 30 driving angles

        # colors
        purple = (155, 0, 155)
        green = (90, 230, 40)
        black = (0, 0, 0)
        white = (255, 255, 255)
        yellow = (255, 255, 0)

        #####Set up screen
        window = pygame.display.set_mode((imageWidth, imageHeight))  # , pygame.FULLSCREEN
        window.fill(black) # hole display is black (=no projection)

        #####Fill screen, if switch is in correct position
        while True:
            if switch:

                #####Initalize Function Variables
                cirCenter = middleAxis  #center of driving radius
                radius = 0.0 # driving radius of middle of robot in meter
                radiusOuter = 0.0 # driving radius of innermost edge of robot in meter
                radiusInner = 0.0 # driving radius of outermost edge of robot in meter

                k = 0 #state of printing

                window.fill(black) # clear projection

                ####Closing program
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        # projection window closing button pressed
                        pygame.quit()
                        sys.exit()

                    elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                        # escape key pressed
                        pygame.quit()
                        sys.exit()

                    elif event.type == pygame.MOUSEBUTTONDOWN:
                        # one of the mousebuttons pressed
                        pygame.quit()
                        sys.exit()

                #to neglect the first bit deviation from zero
                if rotation >= -1 and rotation <= 1:
                    rotation = 0



                ####Compute trajectory

                ###Calculate driving angle
                vLine = pygame.math.Vector2(1, 0) # vector for driving completely right-hand (driving degree = 0)
                vVelRobot = pygame.math.Vector2(-rotation, linear) # vector of current driving
                deg_robot = vLine.angle_to(vVelRobot) # calculation of degree between both vectors (=driving degree)
                deg_robot = int(deg_robot)

                ###No velocity

                if linear < 0:
                    # moving backward is projected and therefore shall not affect data
                    deg_robot = 90
                elif rotation == 0:
                    # no rotation means moving forward (= 90 degree)
                    deg_robot = 90

                ## update data
                deg_robot_array = np.delete(deg_robot_array, 0) # delete oldest data point
                deg_robot_array = np.append(deg_robot_array, deg_robot) # add current data

                if linear > 0 and not rotation == 0:
                    # calculate radius, only if  moving forward (=projecting) and rotating (= no rectangle projection)

                    ### set calculation parameters
                    order = 1
                    fs = 1 / 0.05  # look how long the intervals between the data commands are
                    cutoff = 3 # desired cutoff frequency of the filter, Hz
                    y = butter_lowpass_filter(deg_robot_array, cutoff, fs, order)
                    deg_robot = y[-1] # last data point of array is new driving angle

                    # if radius is too little to be projected, radius is replaced by projectable radius
                    if deg_robot < 55:
                        deg_robot = 55
                    elif deg_robot > 125:
                        deg_robot = 125

                    # new rotational and linear velocity are calculated for radius calculation
                    rot = -math.cos(math.radians(deg_robot))
                    rot = rot* 100
                    rot = int(rot)
                    lin = math.sin(math.radians(deg_robot))
                    lin = lin * 100
                    lin = int(lin)

                    # first step of radius calculation
                    lin_rot = 0
                    if not rot == 0:
                        lin_rot = lin/rot

                    #correcting driving degree
                    if deg_robot < 90:
                        deg = -deg_robot
                    else:
                        deg = 180 - deg_robot

                    # calculate radius for projection
                    if deg_robot <= (90 + angledifmax) and deg_robot >= (90 - angledifmax):
                        # projection of rectangle
                        radius = 0
                    else:
                        # projection of circles/radius
                        radius = lin_rot * pixelPerMeter

                        ###Calculate inner and outer radius
                        if radius > 0:
                            # to the left
                            radiusOuter = radius + 0.5 * robotWidth * pixelPerMeter
                            radiusInner = radius - 0.5 * robotWidth * pixelPerMeter
                            cirCenter = middleAxis - pygame.math.Vector2(radius, 0)

                        else:
                            # to the right
                            radiusOuter = - radius + 0.5 * robotWidth * pixelPerMeter
                            radiusInner = - radius - 0.5 * robotWidth * pixelPerMeter
                            cirCenter = middleAxis - pygame.math.Vector2(radius, 0)


                ###Project on screen
                if not radius == 0:
                    # radius printed
                    pygame.draw.circle(window, yellow, cirCenter, radiusOuter, 0)
                    pygame.draw.circle(window, black, cirCenter, radiusInner, 0)
                    pygame.draw.circle(window, white, cirCenter, radiusOuter, 10)
                    pygame.draw.circle(window, white, cirCenter, radiusInner, 10)
                    k = 1

                elif linear > 0:
                    # rectangle printed
                    pygame.draw.rect(window, yellow, rectArea, 0)
                    pygame.draw.rect(window, white, rectArea, 10)

                    k = 2

                else:
                    k = 3

                #####Update screen
                pygame.display.flip()


                #print state of projection
                if not self.k_old == k:
                    if k == 1:
                        print("Zeigt Drehwinkel")
                    elif k == 2:
                        print("Zeigt Rechteck")
                    else:
                        print("Zeigt nichts")

                self.k_old = k
            else:
                # show nothing
                window.fill(black)
                ####Update screen
                pygame.display.flip()
                time.sleep(0.1)


class SignalThread(threading.Thread):
    def __init__(self, iD, name):
        threading.Thread.__init__(self)
        self.iD = iD
        self.name = name

        # Subscribers
        self.cmd_vel_rotation_subscriber = rospy.Subscriber('/imu', Imu, self.cmd_vel_rotation_callback) # to get current rotational velocity
        self.rc_command_subscriber = rospy.Subscriber('/remote_joy', Joy, self.rc_command_callback) # to get current switch position
        self.cmd_vel_linear_subscriber = rospy.Subscriber('/odom', Odometry, self.cmd_vel_linear_callback) # to get current linear velocity

        # Publisher
        self.debug_publisher = rospy.Publisher('/debug', String, queue_size=1)

        print("init successful")

    def run(self):
        rospy.spin()

    def cmd_vel_linear_callback(self, msg):
        global linear
        linear = msg.twist.twist.linear.x
        linear = linear * 100
        linear = int(linear)

    def cmd_vel_rotation_callback(self, msg):
        global rotation
        rotation = msg.angular_velocity.z
        rotation = rotation * 100
        rotation = int(rotation)

    def rc_command_callback(self, data):
        global switch
        
        # if button C is down or D is up
        if data.buttons[29] == 1 or data.buttons[30] == 1:
            # print only when a state change occurs
            if not switch:
                self.debug_publisher.publish("Projection | Projection activated")
            switch = True
        # if button C is up or D is down
        if data.buttons[28] == 1 or data.buttons[31] == 1:
            if switch:
                self.debug_publisher.publish("Projection | Projection deactivated")
            # reset flag
            switch = False

tProjector = DisplayThread(1, "ProjectorTask")
tSignal = SignalThread(2, "SignalTask")

tProjector.start()
tSignal.start()
