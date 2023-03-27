#!/usr/bin/python

import roslib
import rospy
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty, UInt8

class BebopJoyOverride:

    def __init__(self):
        # Publisher to ardrone cmd_vel topic, can be run in namespace
        self.cmdVelPub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.takeoffPub = rospy.Publisher("takeoff", Empty, queue_size=1)
        self.landPub = rospy.Publisher("land", Empty, queue_size=1)
        self.resetPub = rospy.Publisher("reset", Empty, queue_size=1)
        self.flipPub = rospy.Publisher("flip", UInt8, queue_size=1)

        # Initialize joy and cmd_vel variables
        self.joyData = Joy()
        self.bebopCmdVelReal = Twist() # Publishing to real cmd_vel
        self.bebopCmdVel = Twist() # Subscribing to user cmd_vel
        self.overrideControl = 0
        
        # Load joy parameters
        self.takeoff_index = rospy.get_param("~tello_joy/takeoff_index")
        self.land_index = rospy.get_param("~tello_joy/land_index")
        self.override_index = rospy.get_param("~tello_joy/override_index")
        self.flip_left_index = rospy.get_param("~tello_joy/flip_left_index")
        self.flip_right_index = rospy.get_param("~tello_joy/flip_right_index")
        self.flip_front_index = rospy.get_param("~tello_joy/flip_front_index")
        self.flip_back_index = rospy.get_param("~tello_joy/flip_back_index")

        # Subscriber to joystick topic
        rospy.Subscriber("/slave/joy", Joy, self.JoyCallback, queue_size=1)
        # rospy.Subscriber("cmd_vel", Twist, self.CmdVelCallback, queue_size=1)

    def run(self):
        r = rospy.Rate(50)

        while not rospy.is_shutdown():
            self.cmdVelPub.publish(self.bebopCmdVel)
            r.sleep()

    def JoyCallback(self, data):
        # Assign data to joy variable
        self.joyData = data
        rospy.loginfo_throttle(5.0, "Slave joy data")
        #print self.joyData

        # Setting joy values to be command values for bebop
        self.bebopCmdVel.linear.x = -self.joyData.axes[2]
        self.bebopCmdVel.linear.y = self.joyData.axes[3]
        self.bebopCmdVel.linear.z = self.joyData.axes[1]
        self.bebopCmdVel.angular.z = -self.joyData.axes[0]
        #self.cmdVelPub.publish(self.bebopCmdVelReal)

        # if not self.overrideControl == 0 and self.joyData.buttons[self.takeoff_index] == 1:
        #     rospy.loginfo("[BebopJoyOverride] Takeoff")
        #     self.takeoffPub.publish(Empty())

        # if self.joyData.buttons[self.land_index] == 1:
        #     rospy.loginfo("[BebopJoyOverride] Land")
        #     self.landPub.publish(Empty())

        # if self.joyData.buttons[9] == 1:
        #     rospy.loginfo("[BebopJoyOverride] Reset")
        #     self.resetPub.publish(Empty())

        # if not self.overrideControl == 0 and self.joyData.buttons[self.flip_left_index] == 1:
        #     rospy.loginfo("[BebopJoyOverride] Flip")
        #     flipDirection = 1
        #     self.flipPub.publish(flipDirection)
        # if not self.overrideControl == 0 and self.joyData.buttons[self.flip_right_index] == 1:
        #     rospy.loginfo("[BebopJoyOverride] Flip")
        #     flipDirection = 3
        #     self.flipPub.publish(flipDirection)
        # if not self.overrideControl == 0 and self.joyData.buttons[self.flip_front_index] == 1:
        #     rospy.loginfo("[BebopJoyOverride] Flip")
        #     flipDirection = 0
        #     self.flipPub.publish(flipDirection)
        # if not self.overrideControl == 0 and self.joyData.buttons[self.flip_back_index] == 1:
        #     rospy.loginfo("[BebopJoyOverride] Flip")
        #     flipDirection = 2
        #     self.flipPub.publish(flipDirection)

        # self.overrideControl = self.joyData.buttons[self.override_index]

    # def CmdVelCallback(self, msg):
    #     self.bebopCmdVel = msg

if __name__ == "__main__":
    rospy.init_node("BebopJoyOverrideNode")
    joyControl = BebopJoyOverride()
    joyControl.run()

# Takeoff: R2
# Land: L2
# Reset: Start
# Take control: R1 - [4]
