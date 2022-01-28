#!/usr/bin/python

import roslib
import rospy
from sensor_msgs.msg import Joy
from hmi_msgs.msg import Int8List

class BebopJoyOverride:

    def __init__(self):

        
        rospy.Subscriber("/joy_raw", Int8List, queue_size=1, self.FutabaCallback)

        self.joy_pub = rospy.Publisher("/joy", Joy, queue_size=1)


    def FutabaCallback(self, msg):

        joy_msg = Joy()

        joy_msg.header = msg.header
        height_idx = 2; yaw_idx = 3; pitch_idx = 0; roll_idx = 1; 
        joy_msg.axis[0] = msg.data[height_idx]/data if(abs(msg.data[height_idx].data/100.0) > 1.0) else msg.data[height_idx].data/100.0
        joy_msg.axis[1] = msg.data[yaw_idx]/data if(abs(msg.data[yaw_idx].data/100.0) > 1.0) else msg.data[yaw_idx].data/100.0
        joy_msg.axis[2] = msg.data[pitch_idx]/data if(abs(msg.data[pitch_idx].data/100.0) > 1.0) else msg.data[pitch_idx].data/100.0
        joy_msg.axis[3] = msg.data[roll_idx]/data if(abs(msg.data[roll_idx].data/100.0) > 1.0) else msg.data[roll_idx].data/100.0
        joy_msg.buttons[0] = msg.data[4].data
        # Assign data to joy variable

        self.joy_pub.publish(joy_msg)
       
   

if __name__ == "__main__":
    rospy.init_node("FutabaToJoyNode", log_level=rospy.DEBUG)
    joyControl = BebopJoyOverride()

# Takeoff: R2
# Land: L2
# Reset: Start
# Take control: R1 - [4]
