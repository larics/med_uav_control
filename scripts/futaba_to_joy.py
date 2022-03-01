#!/usr/bin/python

import roslib
import rospy
from sensor_msgs.msg import Joy
from hmi_msgs.msg import Int8List

class FutabaToJoy:

    def __init__(self):


        rospy.init_node("FutabaToJoyNode", log_level=rospy.INFO)

        rospy.Subscriber("/joy_raw", Int8List, self.FutabaCallback)

        self.joy_pub = rospy.Publisher("/joy", Joy, queue_size=1)

        rospy.loginfo("Starting republisher for joy.")

        rospy.spin()


    def FutabaCallback(self, msg):

        joy_msg = Joy()
        joy_msg.axes = [0.0, 0.0, 0.0, 0.0]
        joy_msg.buttons = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # buttons are so long because of joy override, takeoff and stuff like that

        # indexes in joy msg
        joy_yaw_idx = 0; joy_height_idx = 1; joy_roll_idx = 2; joy_pitch_idx = 3; 
        # indexes in rc_msg
        pitch_idx = 1; roll_idx = 0; yaw_idx = 3; height_idx = 2;

        # futaba has some inverted channels
        inverted_ch = -1;   

        # axes, no calibration step
        joy_msg.header = msg.header
        joy_msg.axes[joy_yaw_idx]       = msg.data[yaw_idx]/abs(msg.data[yaw_idx])          if(abs(msg.data[yaw_idx].data/100.0) > 1.0)     else msg.data[yaw_idx].data/100.0        
        joy_msg.axes[joy_height_idx]    = msg.data[height_idx]/abs(msg.data[height_idx])    if(abs(msg.data[height_idx].data/100.0) > 1.0)  else msg.data[height_idx].data/100.0 
        joy_msg.axes[joy_pitch_idx]     = msg.data[pitch_idx]/abs(msg.data[pitch_idx])      if(abs(msg.data[pitch_idx].data/100.0) > 1.0)   else msg.data[pitch_idx].data/100.0
        joy_msg.axes[joy_roll_idx]      = msg.data[roll_idx]/abs(msg.data[roll_idx])        if(abs(msg.data[roll_idx].data/100.0) > 1.0)    else msg.data[roll_idx].data/100.0
        
        # buttons
        joy_msg.buttons[0] = msg.data[4].data

        # Some channels are inverted (pitch, yaw)
        joy_msg.axes[joy_yaw_idx]   *= inverted_ch; 
        joy_msg.axes[joy_pitch_idx] *= inverted_ch; 

        rospy.logdebug(joy_msg)

        self.joy_pub.publish(joy_msg)       
   

if __name__ == "__main__":
    joyControl = FutabaToJoy()

# Takeoff: R2
# Land: L2
# Reset: Start
# Take control: R1 - [4]
