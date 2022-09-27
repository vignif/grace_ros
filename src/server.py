#!/usr/bin/env python

from __future__ import print_function
from grace.grace import Interaction, Agent, FeatureHandler, ProximityFeature, GazeFeature, run


from grace_ros.srv import GetEngagement
import rospy
from std_msgs.msg import Float32
from dynamic_reconfigure.server import Server
from grace_ros.cfg import EngConfig

import logging

eng_logger = logging.getLogger("grace.grace")
eng_logger.setLevel(logging.WARNING)


class GraceServer:
    def __init__(self):
        self.service = rospy.Service(
            "get_engagement", GetEngagement, self.handle_engagement)
        self.srv = Server(EngConfig, self.callback)
        self.pub = rospy.Publisher('mutual_engagement', Float32, queue_size=10)
        self.eng = 0.0

        self.prox_epsilon = 0.0
        self.prox_weight = 1.0
        self.gaze_weight = 1.0

    def callback(self, config, level):
        self.prox_epsilon = config["proxemics_epsilon"]
        self.prox_weight = config["proxemics_weight"]
        self.gaze_weight = config["gaze_weight"]
        rospy.loginfo(
            f"Reconfigure Request: {self.prox_epsilon}, {self.prox_weight}, {self.gaze_weight}")
        return config

    def handle_engagement(self, req):
        # rospy.loginfo("Received request")
        positionA = [req.A.pose.position.x,
                     req.A.pose.position.y, req.A.pose.position.z]
        orientationA = [req.A.pose.orientation.x, req.A.pose.orientation.y,
                        req.A.pose.orientation.z, req.A.pose.orientation.w]

        positionB = [req.B.pose.position.x,
                     req.B.pose.position.y, req.B.pose.position.z]
        orientationB = [req.B.pose.orientation.x, req.B.pose.orientation.y,
                        req.B.pose.orientation.z, req.B.pose.orientation.w]

        AgentA = Agent(req.A.name, positionA, orientationA)
        AgentB = Agent(req.B.name, positionB, orientationB)
        # prox_weight = rospy.get_param("proxemics_weight")
        # gaze = rospy.get_param("gaze_weight")

        ### CALL ENGAGEMENT LIBRARY FROM HERE ###

        rospy.loginfo_once(f'Social Agent name A: {AgentA.name}')
        rospy.loginfo_once(f'Social Agent name B: {AgentB.name}')

        P = ProximityFeature(AgentA, AgentB)
        P.epsilon = self.prox_epsilon
        G = GazeFeature(AgentA, AgentB)

        # feature handler
        F = FeatureHandler(AgentA, AgentB)
        F.add(P, self.prox_weight)
        F.add(G, self.gaze_weight)
        F.compute()
        I = Interaction(F)
        self.eng = I.compute()
        rospy.loginfo(f'Mutual Engagement: {self.eng}')
        self.pub.publish(self.eng)
        return self.eng


if __name__ == "__main__":
    rospy.init_node('system_hri')
    rospy.loginfo("Ready to compute mutual engagement")
    GraceServer()

    rospy.spin()
