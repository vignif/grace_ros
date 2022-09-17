#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from engagement_system.srv import *
from engagement_system.msg import Agent
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
import numpy as np


def client_engagement(A, B):
    rospy.wait_for_service("get_engagement")
    try:
        compute_engagement = rospy.ServiceProxy("get_engagement", GetEngagement)
        resp1 = compute_engagement(A, B)
        return resp1.engagement
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)


if __name__ == "__main__":

    A = Agent()
    B = Agent()
    A.name = "Robot"
    euler_A = np.array([0, 0, 0])
    A.Pose = Pose(
        position=Point(0, 0, 0),
        orientation=Quaternion(*quaternion_from_euler(*euler_A)),
    )

    B.name = "Human"
    euler_B = np.array([0, 0, np.pi])
    B.Pose = Pose(
        position=Point(0, 2, 0),
        orientation=Quaternion(*quaternion_from_euler(*euler_B)),
    )
    client_engagement(A, B)
