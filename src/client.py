#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from grace_ros.srv import *
from grace_ros.msg import Agent
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
import numpy as np


import tf2_msgs.msg
import geometry_msgs.msg
import tf


def client_engagement(A, B):
    rospy.wait_for_service("get_engagement")
    try:
        compute_engagement = rospy.ServiceProxy(
            "get_engagement", GetEngagement)
        resp1 = compute_engagement(A, B)
        return resp1.engagement
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)


def broadcast_tf(agent: Agent):

    pub_tf = rospy.Publisher(
        '/tf', tf2_msgs.msg.TFMessage, queue_size=1)

    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = "world"
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = agent.name

    t.transform.translation.x = agent.Pose.position.x
    t.transform.translation.y = agent.Pose.position.y
    t.transform.translation.z = agent.Pose.position.z

    t.transform.rotation.x = agent.Pose.orientation.x
    t.transform.rotation.y = agent.Pose.orientation.y
    t.transform.rotation.z = agent.Pose.orientation.z
    t.transform.rotation.w = agent.Pose.orientation.w

    tfm = tf2_msgs.msg.TFMessage([t])
    pub_tf.publish(tfm)


def run_demo():
    dx = 0.0
    while not rospy.is_shutdown():

        A = Agent()
        B = Agent()

        A.name = "robot"
        euler_A = np.array([0, 0, 0])
        A.Pose = Pose(
            position=Point(0, 0, 0),
            orientation=Quaternion(*quaternion_from_euler(*euler_A)),
        )

        B.name = "human"
        euler_B = np.array([0, 0, np.pi])
        dx += 0.01
        B.Pose = Pose(
            position=Point(dx, 2, 0),
            orientation=Quaternion(*quaternion_from_euler(*euler_B)),
        )
        engagement = client_engagement(A, B)
        broadcast_tf(A)
        broadcast_tf(B)


if __name__ == "__main__":
    rospy.init_node('demo_client')
    run_demo()

    rospy.spin()
