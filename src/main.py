#!/usr/bin/python3.6
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from nav_msgs.msg import Path, Odometry
from functions import quaternion_to_euler_angle

# set variables
max_angle = 15
laser_msg = LaserScan()
realsense_msg = LaserScan()
pitch_angle = 0

def callback_odom(msg):
    euler = quaternion_to_euler_angle(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)
    global pitch_angle
    pitch_angle = euler[1]
    print(pitch_angle)

def callback_img(msg):
    global realsense_msg
    realsense_msg = msg

def callback_scan(msg):
    global laser_msg
    laser_msg = msg

def main():
    rospy.init_node('main', anonymous = True)
    rate = rospy.Rate(30) # 30hz
    sub_odom = rospy.Subscriber("odom_t265_sync", Odometry, callback_odom)
    sub_img = rospy.Subscriber("scan2", LaserScan, callback_img)
    sub_scan = rospy.Subscriber("scan", LaserScan, callback_scan)
    pub = rospy.Publisher('XD', LaserScan, queue_size=2)

    rospy.loginfo("main node is running")

    while not rospy.is_shutdown():
        if abs(pitch_angle) > max_angle:
            pub.publish(realsense_msg)
        else:
            pub.publish(laser_msg)
        print(pitch_angle)
        rate.sleep()

if __name__ == '__main__':
    main()
