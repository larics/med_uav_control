#!/usr/bin/python

import roslib
import rospy
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty

class BebopJoyOverride:

    def __init__(self):
        # Publisher to ardrone cmd_vel topic, can be run in namespace
        self.cmdVelPub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.takeoffPub = rospy.Publisher("takeoff", Empty, queue_size=1)
        self.landPub = rospy.Publisher("land", Empty, queue_size=1)
        self.resetPub = rospy.Publisher("reset", Empty, queue_size=1)

        # Initialize joy and cmd_vel variables
        self.joyData = Joy()
        self.hpeJoyData = Joy()
        self.hpeJoyData.axes = [0, 0, 0, 0]
        self.hpeCmdVel = Twist() # Publishing to real cmd_vel
        self.bebopCmdVelReal = Twist() # Subscribing to user cmd_vel
        self.overrideControl = 0
        self.hpeOverrideControl = 0
        self.scale_fact = 0.25
        
        # Load joy parameters
        self.takeoff_index = rospy.get_param("~bebop_joy/takeoff_index")
        self.land_index = rospy.get_param("~bebop_joy/land_index")
        self.override_index = rospy.get_param("~bebop_joy/override_index")
        self.hpe_override_index = rospy.get_param("~bebop_joy/hpe_override_index")

        # Subscriber to joystick topic
        rospy.Subscriber("/joy", Joy, self.JoyCallback, queue_size=1)
        rospy.Subscriber("/hpe_joy", Joy, self.hpeJoyCallback, queue_size=1)
        rospy.Subscriber("cmd_vel", Twist, self.CmdVelCallback, queue_size=1)


    def run(self):
        r = rospy.Rate(50)

        while not rospy.is_shutdown():
            if self.overrideControl == 0:
                rospy.loginfo_throttle(5.0, "[BebopJoyOverride] hpe ON")
                self.cmdVelPub.publish(self.hpeCmdVel)
            else:
                rospy.loginfo_throttle(5.0, "[BebopJoyOverride] joystick ON")
                self.cmdVelPub.publish(self.bebopCmdVelReal)



            r.sleep()

    def hpeJoyCallback(self, data): 
        self.hpeJoyData = data

        #rospy.loginfo("Entered callback!")
         # Setting joy values to be command values for bebop
        self.hpeCmdVel.linear.x = self.hpeJoyData.axes[3] * self.scale_fact
        self.hpeCmdVel.linear.y = self.hpeJoyData.axes[2] * self.scale_fact
        self.hpeCmdVel.linear.z = self.hpeJoyData.axes[1] * self.scale_fact
        self.hpeCmdVel.angular.z = self.hpeJoyData.axes[0] * self.scale_fact


    def JoyCallback(self, data):
        # Assign data to joy variable
        self.joyData = data

        # Setting joy values to be command values for bebop
        self.bebopCmdVelReal.linear.x = self.joyData.axes[3] * self.scale_fact
        self.bebopCmdVelReal.linear.y = self.joyData.axes[2] * self.scale_fact
        self.bebopCmdVelReal.linear.z = self.joyData.axes[1] * self.scale_fact
        self.bebopCmdVelReal.angular.z = self.joyData.axes[0] * self.scale_fact

        if not self.overrideControl == 0 and self.joyData.buttons[self.takeoff_index] == 1:
            rospy.loginfo("[BebopJoyOverride] Takeoff")
            self.takeoffPub.publish(Empty())

        if self.joyData.buttons[self.land_index] == 1:
            rospy.loginfo("[BebopJoyOverride] Land")
            self.landPub.publish(Empty())

        if self.joyData.buttons[9] == 1:
            rospy.loginfo("[BebopJoyOverride] Reset")
            self.resetPub.publish(Empty())

        # Override flags
        self.overrideControl = self.joyData.buttons[self.override_index]

    def CmdVelCallback(self, msg):
        self.bebopCmdVel = msg


if __name__ == "__main__":
    rospy.init_node("BebopJoyOverrideNode")
    joyControl = BebopJoyOverride()
    joyControl.run()

# Takeoff: R2
# Land: L2
# Reset: Start
# Take control: R1 - [4]
