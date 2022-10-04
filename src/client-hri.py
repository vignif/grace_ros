#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import tf2_ros
from grace_ros.srv import GetEngagement
from grace_ros.msg import Agent
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
import numpy as np
from hri_msgs.msg import IdsList

import tf

class GraceClientHRI:
    def __init__(self):
        self.sub = rospy.Subscriber('/humans/faces/tracked', IdsList, self.get_id)
        self.runner = rospy.Timer(rospy.Duration(0.1), self.run)
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListen = tf2_ros.TransformListener(self.tfBuffer)

        self.human_tf = None
        self.human_id = None
        self.robot_tf = None

    def get_transformation(source_frame, target_frame,
                        tf_cache_duration=2.0):
        tf_buffer = tf2_ros.Buffer(rospy.Duration(tf_cache_duration))
        tf2_ros.TransformListener(tf_buffer)

        # get the tf at first available time
        try:
            transformation = tf_buffer.lookup_transform(target_frame,
                    source_frame, rospy.Time(0), rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logerr('Unable to find the transformation from %s to %s'
                        % source_frame, target_frame)
        return transformation

    @staticmethod
    def client_engagement(A, B):
        rospy.wait_for_service("get_engagement")
        try:
            compute_engagement = rospy.ServiceProxy(
                "get_engagement", GetEngagement)
            resp1 = compute_engagement(A, B)
            return resp1.engagement
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)


    def get_id(self, data):
        if len(data.ids) > 0:
            self.human_id = data.ids[0]
            rospy.loginfo_throttle_identical(60, f'Face id: {self.human_id}')
            self.human_tf = self.tfBuffer.lookup_transform('camera_color_optical_frame', 'face_'+str(self.human_id), rospy.Time(0), rospy.Duration(1.0))
            self.robot_tf = self.tfBuffer.lookup_transform('camera_color_optical_frame', 'camera_link', rospy.Time(0), rospy.Duration(1.0))
            
        if self.human_tf is None or self.robot_tf is None:
            rospy.logerr_throttle(2.0, 'No human or robot tf')


    def run(self, event):
        rospy.loginfo_once("Running GraceClientHRI")

        if self.robot_tf is not None and self.human_tf is not None:
            robot_position = self.robot_tf.transform.translation
            robot_orientation = self.robot_tf.transform.rotation

            human_position = self.human_tf.transform.translation
            human_orientation = self.human_tf.transform.rotation

            robot = Agent()
            robot.name = "robot"
            robot.pose = Pose()
            robot.pose.position = robot_position
            robot.pose.orientation = Quaternion(x=robot_orientation.x, y=robot_orientation.y, z=robot_orientation.z, w=robot_orientation.w)

            human = Agent()
            human.name = "human"
            human.pose = Pose()
            human.pose.position = human_position
            human.pose.orientation = Quaternion(x=human_orientation.x, y=human_orientation.y, z=human_orientation.z, w=human_orientation.w)

            engagement = self.client_engagement(robot, human)
            
            rospy.loginfo_throttle(1, f'Engagement: {engagement}')




if __name__ == "__main__":
    rospy.init_node('grace_client_hri')
    GraceClientHRI()
    rospy.spin()
