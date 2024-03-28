import RPi.GPIO as GPIO
import rospy
import time
import sys
import threading
import queue
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy


# Set the GPIO mode
GPIO.setmode(GPIO.BCM)

# Set the GPIO pin used for the servo
servo_pin = 18
GPIO.setup(servo_pin, GPIO.OUT)

# Set the PWM frequency to 50 Hz
pwm = GPIO.PWM(servo_pin, 50)

# Start PWM with a duty cycle of 0%
pwm.start(0)

# Start node
rospy.init_node('RaspPi', anonymous=False, log_level=rospy.DEBUG) # Initalize ROS-Node

class PubSubThread(threading.Thread):
    def __init__(self, iD, name):
        threading.Thread.__init__(self)
        self.iD = iD
        self.name = name
        
        # Create subscribers
        self.cmd_vel_rotation_subscriber = rospy.Subscriber('/imu', Imu, self.cmd_vel_rotation_callback) # to get current rotational velocity
        self.rc_command_subscriber = rospy.Subscriber('/remote_joy', Joy, self.rc_command_callback) # to get current switch position
        self.cmd_vel_linear_subscriber = rospy.Subscriber('/odom', Odometry, self.cmd_vel_linear_callback) # to get current linear velocity
        
        # Create publisher
        self.pub_debug = rospy.Publisher('/debug', String, queue_size=1)

    def run(self):
        global stop_flag  # wait, time
        while True:
            # print(zone)
            if stop_flag:
                time.sleep(2)

                # Stop the subscribers
                self.stop_subscribers()

                # Reset Pi
                pwm.stop()
                GPIO.cleanup()

                # Exit the program
                sys.exit()

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
        
        # Button B set to 1
        if data.buttons[6] == 1:
            # Only print the message if the switch has just changed
            if not switch:
                self.debug_publisher.publish("RaspBi | Projection activated due to B")
                print("Projektion an wegen B")
                # Set the switch boolean to True
                switch = True

        # Button B not 1
        else:
            # Button ... set to 1
            if data.buttons[29] == 1:
                # Only print the message if the switch has just changed
                if not switch:
                    self.debug_publisher.publish("RaspBi | Projection activated")
                    switch = True
                    print("Projektion an")
            # Button ... set to 1
            elif data.buttons[28] == 1:
                # Only print the message if the switch has just changed
                if switch:
                    self.debug_publisher.publish("RaspBi | Projection deactivated")
                    switch = False
                    print("Projektion aus")

    def stop_subscribers(self):
        self.cmd_vel_rotation_subscriber.unregister()
        self.rc_command_subscriber.unregister()
        self.cmd_vel_linear_subscriber.unregister()

class ServoThread(threading.Thread):
    def __init__(self, iD, name):
        threading.Thread.__init__(self)
        self.iD = iD
        self.name = name

    def run(self):
        global linear, rotation, switch, stop_flag

        # Init variables
        min_angle =
        max_angle = 
        min_duty = 
        max_duty = 
        target_angle = 
        prev_duty_cycle = 
        tolerance = 

        # Set the initial duty cycle to 0
        pwm.ChangeDutyCycle(0)


        while not stop_flag:
            if switch:
                # Set the duty cycle
                duty_cycle = (target_angle - min_angle) * (max_duty - min_duty) / (max_angle - min_angle)

                # Change the duty cycle
                if abs(duty_cycle - prev_duty_cycle) > tolerance:
                    pwm.ChangeDutyCycle(duty_cycle)

                    # save the current duty cycle
                    prev_duty_cycle = duty_cycle

# initalize threads
tPubSub = PubSubThread(1, "PubSubTask")
tServo = ServoThread(2, "ServoTask")

# start threads
tPubSub.start()
tServo.start()

print("Threads started")