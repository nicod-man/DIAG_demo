# goto action

import time, math, sys, random

import rospy
import tf

import actionlib
from actionlib_msgs.msg import *

from geometry_msgs.msg import Twist, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


from actionproxy import ActionProxy

ACTION_NAME = 'sense'
TOPIC_amcl_pose = 'amcl_pose'   # localizer pose
TOPIC_odom = 'odom'


'''
Sense the value of a fluent

'''

sys.path.append("../fluents")  # fluents folder
from lightcolor_fluentproxy import LightColorFluentProxy
from open_fluentproxy import OpenFluentProxy
from personhere_fluentproxy import PersonHereFluentProxy
from mypersonhere_fluentproxy import MyPersonHereFluentProxy
from facedetected_fluentproxy import FaceDetectionFluentProxy
from legdetection_fluentproxy import LegDetectorFluentProxy

class SenseActionProxy(ActionProxy):

    def __init__(self, actionname):
        ActionProxy.__init__(self, actionname)

    def __del__(self):
        ActionProxy.__del__(self)

    def action_thread(self, params):

        # which fluent must be sensed
        v = params.split('_')
        sensingfluent = v[0]

        if sensingfluent=='lightcolor':
            l = LightColorFluentProxy(sensingfluent, rosnode=False)
            l.sensingStep()

        elif sensingfluent=='open':
            l = OpenFluentProxy(sensingfluent, rosnode=False)
            l.sensingStep()

        elif sensingfluent=='personhere':
            l = PersonHereFluentProxy(sensingfluent, rosnode=False)
            l.sensingStep()

        elif sensingfluent=='mypersonhere':
            l = MyPersonHereFluentProxy(sensingfluent, rosnode=False)
            l.sensingStep()

        elif sensingfluent=='facedetection':
            l = FaceDetectionFluentProxy(sensingfluent, rosnode=False)
            l.sensingStep()

        elif sensingfluent=='legdetection':
            l = LegDetectorFluentProxy(sensingfluent, rosnode=False)
            l.sensingStep()

        else:
            print("ERROR !!! Cannot sense value of fluent %s !!!" %(sensingfluent))


if __name__ == "__main__":

    params = None
    if (len(sys.argv)>1):
        params = sys.argv[1]

    a = SenseActionProxy(ACTION_NAME)
    
    if params is not None:
        a.execute(params)  # blocking, CTRL-C to interrupt
    else:
        a.run_server()     # blocking, CTRL-C to interrupt

