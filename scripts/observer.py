#!/usr/bin/python

import roslib
import rospy
import rosbag
import datetime
import copy
from std_msgs.msg import Float32, Time
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


        # Subscribers
        rospy.Subscriber("/bebop/odometry", Odometry, self.OdometryCallback, queue_size=1)
        self.end_time_pub   = rospy.Publisher("/end_time",    Time, queue_size=1)
        self.start_time_pub = rospy.Publisher("/start_time",  Time, queue_size=1)


        self.current_pos_x = 0; self.current_pos_y = 0; 
        # Define goal pose
        self.goal_pos_x = 9.0
        self.timeout = 120
        # Y limits 
        self.y_upper_limit = -1.0
        self.y_lower_limit = -3.0
        self.x_upper_limit = 1.0 
        self.x_lower_limit = -1.0
        
        # Start time
        self.start_time = rospy.Time.now().to_sec(); now = datetime.datetime.now(); 


        sleep_time = 5
        rospy.sleep(sleep_time)

        # Current rate
        self.rate = rospy.Rate(10)
        self.odom_recv = False; 
        self.start_published = False; self.end_published = False
        self.initialized = True


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
                
                # Start condition
                start_condition = True if (self.in_range(self.current_pos_x, self.x_upper_limit, self.x_lower_limit) and self.current_pos_y > 0) else False
                
                # Check if goal line has been surpassed
                end_condition = True if (self.in_range(self.current_pos_y, self.y_upper_limit, self.y_lower_limit) and self.current_pos_x > self.goal_pos_x) else False

                # Check how much time has passed
                time_elapsed = rospy.Time.now().to_sec() - self.start_time

                #rospy.logdebug("start_condition: {}".format(start_condition))
                #rospy.logdebug("end_condition: {}".format(end_condition))
                
                rospy.logdebug("elapsed: {}".format(time_elapsed))
                rospy.logdebug("{}".format(convert(time_elapsed)))

                if start_condition and not self.start_published: 
                    
                    current_time_msg = Time()
                    current_time_msg.data = rospy.Time.now()
                    self.start_time_pub.publish(current_time_msg)
                    self.start_published = True

                if end_condition and not self.end_published: 

                    current_time_msg = Time()
                    current_time_msg.data = rospy.Time.now()
                    self.end_time_pub.publish(current_time_msg)
                    self.end_published = True

                    # TODO: Add elapsed time for the counter 

                if (time_elapsed > self.timeout) or end_condition: 

                    exit()

                self.rate.sleep()

            self.bag.close()
                
            exit()              


def convert(seconds):
    seconds = seconds % (24 * 3600)
    hour = seconds // 3600
    seconds %= 3600
    minutes = seconds // 60
    seconds %= 60
     
    return "%d:%02d:%02d" % (hour, minutes, seconds)


if __name__ == "__main__": 

    rospy.init_node("observer", log_level=rospy.DEBUG)
    obs = Observer()
    obs.run()



