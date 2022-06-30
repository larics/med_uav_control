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
        self.bebopCmdVelReal = Twist() # Publishing to real cmd_vel
        self.bebopCmdVel = Twist() # Subscribing to user cmd_vel
        self.overrideControl = 0
        self.hpeOverrideControl = 0
        
        # Load joy parameters
        self.takeoff_index = rospy.get_param("~bebop_joy/takeoff_index")
        self.land_index = rospy.get_param("~bebop_joy/land_index")
        self.override_index = rospy.get_param("~bebop_joy/override_index")
        self.hpe_override_index = rospy.get_param("~bebop_joy/hpe_override_index")

        # Subscriber to joystick topic
        rospy.Subscriber("/joy", Joy, self.JoyCallback, queue_size=1)
        rospy.Subscriber("/hpe_joy", Joy, self.JoyCallback, queue_size=1)
        rospy.Subscriber("cmd_vel", Twist, self.CmdVelCallback, queue_size=1)


    def run(self):
        r = rospy.Rate(50)

        while not rospy.is_shutdown():
            if self.overrideControl == 0:
                rospy.loginfo_throttle(5.0, "[BebopJoyOverride] override OFF")
                self.cmdVelPub.publish(self.bebopCmdVel)
            else:
                rospy.loginfo_throttle(5.0, "[BebopJoyOverride] override ON")
                self.cmdVelPub.publish(self.bebopCmdVelReal)

            if self.hpeOverrideControl == 1.0: 
                rospy.loginfo_throttle(1.0, "[BebopJoyOverride] override HPE-ON")

            r.sleep()

    def hpeJoyCallback(self, data): 
        self.hpeJoyData = data

    def JoyCallback(self, data):
        # Assign data to joy variable
        self.joyData = data

        # Scale fact for slower control 
        scale_fact = 0.25

        # If hpe_override button pressed, take cmds from hpe!
        if self.hpeOverrideControl: 
            self.joyData.axes = self.hpeJoyData.axes 

        # Setting joy values to be command values for bebop
        self.bebopCmdVelReal.linear.x = self.joyData.axes[3] * scale_fact
        self.bebopCmdVelReal.linear.y = self.joyData.axes[2] * scale_fact
        self.bebopCmdVelReal.linear.z = self.joyData.axes[1] * scale_fact
        self.bebopCmdVelReal.angular.z = self.joyData.axes[0] * scale_fact

        if not self.overrideControl == 0 and self.joyData.buttons[self.takeoff_index] == 1:
            rospy.loginfo("[BebopJoyOverride] Takeoff")
            self.takeoffPub.publish(Empty())

        if self.joyData.buttons[self.land_index] == 1:
            rospy.loginfo("[BebopJoyOverride] Land")
            self.landPub.publish(Empty())

        if self.joyData.buttons[9] == 1:
            rospy.loginfo("[BebopJoyOverride] Reset")
            self.resetPub.publish(Empty())

        # If HPE override button press, send command from HPE
        if self.hpeOverrideControl == 1: 
            rospy.loginfo("[BebopJoyOverride] Hpe control active!")
            self.cmdVelPub.publish(self.hpeCmdVel)
        # If not, send command from joy
        else:
            rospy.loginfo("[BebopJoyOverride] Joy control active!")
            self.cmdVelPub.publish(self.bebopCmdVel)

        # Override flags
        self.overrideControl = self.joyData.buttons[self.override_index]
        self.hpeOverrideControl = self.joyData.buttons[self.hpe_override_index]

    def CmdVelCallback(self, msg):
        self.bebopCmdVel = msg

    def HpeCmdVelCallback(self, msg): 
        self.hpeCmdVel = msg

if __name__ == "__main__":
    rospy.init_node("BebopJoyOverrideNode")
    joyControl = BebopJoyOverride()
    joyControl.run()

# Takeoff: R2
# Land: L2
# Reset: Start
# Take control: R1 - [4]
