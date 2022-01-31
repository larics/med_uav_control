#!/usr/bin/python

import roslib
import rospy
import rosbag
import datetime
import copy
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry

# https://www.javatpoint.com/how-to-create-a-dataframes-in-python
# Rosbag cookbok http://wiki.ros.org/rosbag/Cookbook
# https://answers.ros.org/question/52773/record-with-rosbag-from-launch-file/

# TODO: 
# - Add exporting to CSV
# - Add timer and watching time 
# - Add sucessful recording of full bag file 

class Observer: 

    def __init__(self): 

        rospy.init_node("observer", log_level=rospy.INFO)

        # Publishers
        self.duration_pub = rospy.Publisher("/duration", Float32, queue_size=1)

        # Subscribers
        rospy.Subscriber("/bebop/odometry", Odometry, self.OdometryCallback, queue_size=1)
        rospy.Subscriber("/joy", Joy, self.JoyCallback, queue_size=1)
        rospy.Subscriber("/duration", Float32, self.DurationCallback, queue_size=1)

        self.current_pos_x = 0; self.current_pos_y = 0; 
        # Define goal pose
        self.goal_pos_x = 9.0
        self.duration_t_sec = 120
        # Y limits 
        self.upper_limit = -1.0;  self.lower_limit = -3.0;
        # Start time
        self.start_time = rospy.Time.now().to_sec(); now = datetime.datetime.now(); 
        # Current date
        date = "{}-{}-{}-{}-{}".format(now.day, now.month, now.year, now.hour, now.minute)

        sleep_time = 10
        rospy.sleep(sleep_time)
        # Bag name
        self.bag = rosbag.Bag('/home/developer/catkin_ws/src/med_uav_control/experiments/{}.bag'.format(date), 'w')
        # Current rate
        self.rate = rospy.Rate(5)

        self.odom_recv = False; self.joy_recv = False
        self.initialized = True

        self.duration_pub.publish(Float32(0.0))

    

    def DurationCallback(self, msg):

        self.bag.write("/duration", msg)

    
    def JoyCallback(self, msg):

        self.joy_recv = True

        self.joy_msg = Joy()
        self.joy_msg.header = msg.header
        self.joy_msg.axes = msg.axes
        self.joy_msg.buttons = msg.buttons


    def OdometryCallback(self, msg): 

        self.odom_recv = True

        self.current_pos_x = msg.pose.pose.position.x
        self.current_pos_y = msg.pose.pose.position.y

        self.odom_msg = Odometry()
        self.odom_msg.header = msg.header
        self.odom_msg.pose = msg.pose
        self.odom_msg.twist = msg.twist

    def in_range(self, pos, upper_limit, lower_limit): 
 
        if pos > lower_limit and pos < upper_limit: 
            return True
        else:
            return False

    def run(self): 
        
        
        rospy.loginfo("Entered run!")

        if not self.initialized:
            rospy.sleep(0.1)
            
        else:
        
            while True: 
                
                # Check if goal line has been surpassed
                end_condition = True if (self.in_range(self.current_pos_y, self.upper_limit, self.lower_limit) and self.current_pos_x > self.goal_pos_x) else False

                # Check how much time has passed
                time_elapsed = rospy.Time.now().to_sec() - self.start_time

                rospy.loginfo("end_condition: {}".format(end_condition))

                # Check end_condition
                if (not end_condition):

                    if time_elapsed > self.duration_t_sec:
                        timeout = True
                    else:
                        timeout = False
                    
                    # Write odometry message and joy message to bag 
                    if self.odom_recv and self.joy_recv:
                        odom_msg = copy.deepcopy(self.odom_msg); joy_msg = copy.deepcopy(self.joy_msg)
                        self.bag.write("/bebop/odometry", odom_msg)
                        rospy.loginfo("...")
                        self.bag.write("/joy", joy_msg)

                else:

                    if timeout:
                        successful = False
                        
                        rospy.loginfo("Timeout reached!")

                    else:
                        successful = True

                        rospy.loginfo("Labyrinth has been finished!")
                        rospy.loginfo("Duration is: {} secs".format(time_elapsed))

                        self.duration_pub.publish(Float32(time_elapsed))

                    break

                self.rate.sleep()


            self.bag.close()
                
            exit()              
                  

if __name__ == "__main__": 

    obs = Observer()
    obs.run()


