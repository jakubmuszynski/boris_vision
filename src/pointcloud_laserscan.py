#!/usr/bin/python
import rospy
import math
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from functions import quaternion_to_euler_angle

def callback_odom(msg):
    euler = quaternion_to_euler_angle(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)
    global pitch_angle
    pitch_angle = euler[1]

def callback_img(msg):
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    height, width = img.shape

    pitch_rad = math.radians(pitch_angle)
    tan_pitch = math.tan(pitch_rad)
    offset = 2 #target image height = offset * 2
    camera_constant_angle = 21.2 #calculated from D435 datasheet for 640x480 resolution
    camera_constant_rad = math.radians(camera_constant_angle)
    camera_constant = (height/2)/math.tan(camera_constant_rad)

    val = int(camera_constant*tan_pitch)
    h = int(height*2/3) #set detection height

    img_final = np.zeros((height, width), np.uint16)
    param = h
    img_crop = img[param-offset:param+offset, 0:width]
    img_final[(height/2)-offset:(height/2)+offset, 0:width] = img_crop
    img_pub = bridge.cv2_to_imgmsg(img_final)
    pub_img.publish(img_pub)

    # if  val < (h-offset) and val > -(h-offset) and val < (height-h-offset):
    #     img_final = np.zeros((height, width), np.uint16)
    #     param = h+val
    #     img_crop = img[param-offset:param+offset, 0:width]
    #     img_final[(height/2)-offset:(height/2)+offset, 0:width] = img_crop
    #     img_pub = bridge.cv2_to_imgmsg(img_final)
    #     pub_img.publish(img_pub)
    # else:
    #     img_final = np.zeros((height, width), np.uint16)
    #     img_pub = bridge.cv2_to_imgmsg(img_final)
    #     pub_img.publish(img_pub)

rospy.init_node('pointcloud_laserscan', anonymous = True)
sub_odom = rospy.Subscriber("odom_t265_sync", Odometry, callback_odom)
sub_img = rospy.Subscriber("align_depth_sync", Image, callback_img)
pub_img = rospy.Publisher("align_depth_crop", Image, queue_size=2)
rate = rospy.Rate(30) # 30hz

bridge = CvBridge()

rospy.loginfo("pointcloud_laserscan is running")

while not rospy.is_shutdown():

    rate.sleep()
