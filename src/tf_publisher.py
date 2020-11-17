#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry

def callback(msg):
    br_central_map = tf.TransformBroadcaster()
    br_central_map.sendTransform((-msg.pose.pose.position.x, msg.pose.pose.position.y, -msg.pose.pose.position.z),
                     (0, 0, 0, 1),
                     rospy.Time.now(),
                     "central",
                     "map")

    br_map_odom = tf.TransformBroadcaster()
    br_map_odom.sendTransform((0, 0, 0),
                      (-msg.pose.pose.orientation.y, msg.pose.pose.orientation.x, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                      rospy.Time.now(),
                      "map",
                      "camera")

    br_odom_cam = tf.TransformBroadcaster()
    br_odom_cam.sendTransform((0, 0, 0),
                           (0, 0, 0, 1),
                      rospy.Time.now(),
                      "camera",
                      "cam_link")

    br_cam_lidar = tf.TransformBroadcaster()
    br_cam_lidar.sendTransform((-0.075813, -0.043789, -0.0709),
                        (0, 0, 0.5, 0.8660254),
                      rospy.Time.now(),
                      "cam_link",
                      "laser")

    br_lidar_robot = tf.TransformBroadcaster()
    br_lidar_robot.sendTransform((-0.075813, -0.043789, -0.0709),
                        (0, 0, 0.5, 0.8660254),
                      rospy.Time.now(),
                      "laser",
                      "base_link")

if __name__ == '__main__':
    rospy.init_node('tf_publisher')
    rospy.Subscriber('odom_t265_sync', Odometry, callback)

    rospy.loginfo("TF publisher is running")

    rospy.spin()
