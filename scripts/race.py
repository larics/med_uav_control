#!/usr/bin/python

import roslib
import rospy
import rosbag
import datetime
import copy
import sqlite3
import sys
from std_msgs.msg import Float32, Time, String
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry

# https://www.javatpoint.com/how-to-create-a-dataframes-in-python
# Rosbag cookbok http://wiki.ros.org/rosbag/Cookbook
# https://answers.ros.org/question/52773/record-with-rosbag-from-launch-file/

# TODO: 
# - Add timer and watching time 
# - Add sucessful recording of full bag file 

class Race: 

    def __init__(self): 


        # Subscribers
        rospy.Subscriber("/bebop/odometry", Odometry, self.OdometryCallback, queue_size=1)
        self.end_time_pub   = rospy.Publisher("/end_time",    Time, queue_size=1)
        self.start_time_pub = rospy.Publisher("/start_time",  Time, queue_size=1)
        self.duration_pub = rospy.Publisher("/duration", String, queue_size=1)

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

        # Open database
        try:
            path_to_db = "/home/developer/catkin_ws/src/med_uav_control/db"
            self.db_conn = sqlite3.connect('{}/test_database.db'.format(path_to_db)) 

        except Exception as e:
            rospy.logwarn("{}".format(str(e)))

        self.c = self.db_conn.cursor()
        # Check if table exists, if not -> create new one
        self.check_if_table_exists(self.c, 'RaceResults')
        self.list_db_table(self.db_conn)


        rospy.loginfo("Table is ready!")
 
        # Current rate
        self.rate = rospy.Rate(10)
        self.odom_recv = False; 
        self.start_published = False; self.end_published = False
        self.initialized = True

    
    def check_if_table_exists(self, cursor, table_name): 

        cursor.execute("SELECT count(name) FROM sqlite_master WHERE type=\'table\' AND name=\'{}\'".format(table_name))

        #if the count is 1, then table exists
        if self.c.fetchone()[0]==1: rospy.loginfo('Database exists!')
        else: self.create_db_table(cursor, table_name)

    def create_db_table(self, cursor, table_name): 

        # Creating table
        table = '''CREATE TABLE {} (
                    Email VARCHAR(255) NOT NULL,
                    Name CHAR(25) NOT NULL,
                    Duration CHAR(25) NOT NULL, 
                    Score INT
                 );'''.format(table_name)

        cursor.execute(table)
        rospy.loginfo("Created database!")    

    def list_db_table(self, connection): 


        cur = connection.cursor()
        cur.execute("SELECT * FROM RaceResults")

        rows = cur.fetchall()

        for row in rows:
            print(row)


    def insert_into_db_table(self, connection, cursor, data):

        #try:
        sqlite_insert_query = """INSERT INTO {}
                            (Email, Name, Duration, Score) 
                            VALUES 
                            ('{}','{}','{}',{})""".format(data["table_name"],
                                                          data["email"],
                                                          data["name"],
                                                          data["duration"],
                                                          data["score"])

        count = cursor.execute(sqlite_insert_query)
        connection.commit()
        rospy.loginfo("Record inserted successfully into SqliteDb_developers table {}".format(cursor.rowcount))
        cursor.close()

        #except sqlite3.Error as error:
        #    print("Failed to insert data into sqlite table", error)
        #finally:
        #    if sqliteConnection:
        #        sqliteConnection.close()
        #        print("The SQLite connection is closed")



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

                # Check how much time has passed --> FIX this after fixing condition
                race_time = rospy.Time.now().to_sec() - self.start_time
                
                rospy.logdebug("race_time: {}".format(convert(race_time)))

                duration_msg = String()
                duration_msg.data = "{}".format(convert(race_time))
                self.duration_pub.publish(duration_msg)

                # Create data for inserting into results base
                data_ = {"table_name": "RaceResults", 
                         "email": "{}".format("fzoric8@gmail.com"), 
                         "name" : "{}".format("racer_name"), 
                         "duration": "{}".format(race_time), 
                         "score": 0}

                # Insert data into table 
                self.insert_into_db_table(self.db_conn, self.c, data_)
                rospy.loginfo("Entered result into DB!")

                if start_condition and not self.start_published: 
                    
                    self.start_time = rospy.Time.now().to_sec()
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

                if (race_time > self.timeout) or end_condition: 
                    
                    self.db_conn.close()
                    exit()

                self.rate.sleep()

            self.db_conn.close()
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
    obs = Race()
    obs.run()



