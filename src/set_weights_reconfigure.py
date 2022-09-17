#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from engagement_system.cfg import EngConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {proxemics_weight_param}, {gaze_weight_param}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("dynamic_tutorials", anonymous = False)

    srv = Server(EngConfig, callback)
    rospy.spin()