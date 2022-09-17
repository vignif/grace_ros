#!/usr/bin/env python

from __future__ import print_function

from engagement_system.srv import GetEngagement
import rospy

from dynamic_reconfigure.server import Server
from engagement_system.cfg import EngConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {proxemics_weight_param}, {gaze_weight_param}""".format(**config))
    return config

def handle_engagement(req):
    AgentA = req.A
    AgentB = req.B
    engagement = 0
    
    prox = rospy.get_param("proxemics_weight")
    gaze = rospy.get_param("gaze_weight")

    ### CALL ENGAGEMENT LIBRARY FROM HERE ###
    
    rospy.loginfo(f'Social Agent name A: {AgentA.name}')
    rospy.loginfo(f'Social Agent name B: {AgentB.name}')

    rospy.loginfo("Engagement request received")
    
    return engagement

def engagement_server():
    rospy.init_node('system_hri')
    s = rospy.Service('get_engagement', GetEngagement, handle_engagement)
    rospy.loginfo("Ready to receive agents of an interaction")
    srv = Server(EngConfig, callback)
    rospy.spin()

if __name__ == "__main__":
    engagement_server()