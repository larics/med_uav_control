#!/usr/bin/python

import roslib
import rospy
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped, Vector3
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty

class BebopJoyOverride:

    def __init__(self):

        self.use_sim_bebop = True
        if self.use_sim_bebop: 
            self.eulerRefPub = rospy.Publisher("euler_ref", Vector3, queue_size=1)
            self.zRefPub = rospy.Publisher("pos_ref", Vector3, queue_size=1)

            rospy.Subscriber("euler_ref", Vector3, queue_size=1)

        else:
            # Publisher to ardrone cmd_vel topic, can be run in namespace
            self.cmdVelPub = rospy.Publisher("cmd_vel_real", Twist, queue_size=1)
            rospy.Subscriber("cmd_vel", Twist, self.CmdVelCallback, queue_size=1)

        self.takeoffPub = rospy.Publisher("takeoff", Empty, queue_size=1)
        self.landPub = rospy.Publisher("land", Empty, queue_size=1)
        self.resetPub = rospy.Publisher("reset", Empty, queue_size=1)

        # Initialize joy and cmd_vel variables
        self.joyData = Joy()
        self.bebopCmdVelReal = Twist() # Publishing to real cmd_vel
        self.bebopCmdVel = Twist() # Subscribing to user cmd_vel
        self.eulerRef = Vector3()
        self.posRef = Vector3()
        self.overrideControl = 0
        
        # Load joy parameters
        self.takeoff_index = rospy.get_param("~bebop_joy/takeoff_index")
        self.land_index = rospy.get_param("~bebop_joy/land_index")
        self.override_index = rospy.get_param("~bebop_joy/override_index")

        # Subscriber to joystick topic
        rospy.Subscriber("/joy", Joy, self.JoyCallback, queue_size=1)


    def run(self):
        r = rospy.Rate(50)

        while not rospy.is_shutdown():
            
            # if we use real bebop we control Twist and cmdVelpub
            if not self.use_sim_bebop:

                if self.overrideControl == 0:
                    rospy.loginfo_throttle(5.0, "[BebopJoyOverride] override OFF")
                    self.cmdVelPub.publish(self.bebopCmdVel)
                else:
                    rospy.loginfo_throttle(5.0, "[BebopJoyOverride] override ON")
                    self.cmdVelPub.publish(self.bebopCmdVelReal)

            # if we use sim bebop we control eulerRef and poseRef (z)
            else:

                if self.overrideControl == 0:
                    rospy.loginfo_throttle(5.0, "[BebopJoyOverride] override OFF")
                    self.eulerRefPub.publish(self.eulerRef)
                    self.zRefPub.publish(self.posRef)
                else:
                    rospy.loginfo_throttle(5.0, "[BebopJoyOverride] override ON")
                    self.eulerRefPub.publish(self.eulerRef)
                    self.zRefPub.publish(self.posRef)
            
            r.sleep()

    def JoyCallback(self, data):
        # Assign data to joy variable
        self.joyData = data
        
        if self.use_sim_bebop:
            
            # Empirically determined scale factors --> larger --> agressive 
            rp_scale_fact = 0.3; yaw_scale_fact = 0.05; z_scale_fact = 0.025; 

            reverse = True
            if reverse:
                reverse_dir = -1
            else:
                reverse_dir = 1


            self.eulerRef.x     = self.joyData.axes[2]  * rp_scale_fact * reverse_dir
            self.eulerRef.y     = self.joyData.axes[3]  * rp_scale_fact 
            self.eulerRef.z     += self.joyData.axes[0] * yaw_scale_fact
            self.posRef.z       += self.joyData.axes[1] * z_scale_fact
            self.eulerRefPub.publish(self.eulerRef)
            
            # Test this part? publish current height maybe, to keep it from losing 
            self.zRefPub.publish(self.posRef)

            debug = True
            if debug: 
                rospy.logdebug("eulerRef.x : {}".format(self.eulerRef.x))
                rospy.logdebug("eulerRef.y : {}".format(self.eulerRef.y))
                rospy.logdebug("eulerRef.z : {}".format(self.eulerRef.z))
                rospy.logdebug("posRef.z : {}".format(self.posRef.z))

        else:

            # Setting joy values to be command values for bebop
            self.bebopCmdVelReal.linear.x = self.joyData.axes[3]
            self.bebopCmdVelReal.linear.y = self.joyData.axes[2]
            self.bebopCmdVelReal.linear.z = self.joyData.axes[1]
            self.bebopCmdVelReal.angular.z = self.joyData.axes[0]
            #self.cmdVelPub.publish(self.bebopCmdVelReal)


        if not self.overrideControl == 0 and self.joyData.buttons[self.takeoff_index] == 1:
            rospy.loginfo("[BebopJoyOverride] Takeoff")
            self.takeoffPub.publish(Empty())

        if self.joyData.buttons[self.land_index] == 1:
            rospy.loginfo("[BebopJoyOverride] Land")
            self.landPub.publish(Empty())

        if self.joyData.buttons[9] == 1:
            rospy.loginfo("[BebopJoyOverride] Reset")
            self.resetPub.publish(Empty())

        self.overrideControl = self.joyData.buttons[self.override_index]

    def CmdVelCallback(self, msg):
        self.bebopCmdVel = msg

    def EulerRefCallback(self, msg):
        self.eulerRef = msg

    def posRefCallback(self, msg): 
        # TODO: Add height and yaw to current value 
        pass

if __name__ == "__main__":
    rospy.init_node("BebopJoyOverrideNode", log_level=rospy.INFO)
    joyControl = BebopJoyOverride()
    joyControl.run()

# Takeoff: R2
# Land: L2
# Reset: Start
# Take control: R1 - [4]
