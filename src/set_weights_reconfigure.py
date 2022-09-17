#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from grace_ros.cfg import EngConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {proxemics_weight}, {gaze_weight}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("dynamic_tutorials", anonymous = False)

    srv = Server(EngConfig, callback)
    rospy.spin()