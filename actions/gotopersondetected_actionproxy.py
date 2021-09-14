# goto action

import time, math, sys, random

import rospy
import tf

import actionlib

from actionlib_msgs.msg import *
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from sensor_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry

from actionproxy import ActionProxy

ACTION_NAME = 'gotopersondetected'
ACTION_move_base = 'move_base'  # ROS action
TOPIC_amcl_pose = 'amcl_pose'  # localizer pose
BASE_POSE_GROUND_TRUTH_TOPIC = '/base_pose_ground_truth'
TARGET_RADIUS = 0.3

LEG_FLUENT_TOPIC = '/human_positions'



class GotoPersonDetectedActionProxy(ActionProxy):

    def __init__(self, actionname):
        ActionProxy.__init__(self, actionname)

        self.map_robot_pose = [0,0,0]
        self.humans = {}

        self.leg_sub = rospy.Subscriber(LEG_FLUENT_TOPIC, PoseArray, self.humans_cb)
        rospy.sleep(0.5)

        #self.loc_sub = rospy.Subscriber(TOPIC_amcl_pose, PoseWithCovarianceStamped, self.localizer_cb)
        self.ac_movebase = None

        self.ac_movebase = actionlib.SimpleActionClient(ACTION_move_base, MoveBaseAction)
        self.ac_movebase.wait_for_server()

    def __del__(self):
        ActionProxy.__del__(self)

    def humans_cb(self,data):
        """
            :param data: it is a PoseArray()

            :return: map that has
                        - key: the id (not really an ID, but a fake id) of a person
                        - values: the relatives position (w.r.t. marrtino).
                     Note that the orientation is already in radians, and we do not need
                     to convert it.
        """

        for i,obj in enumerate(data.poses):
            self.humans[i] = [obj.position.x,obj.position.y,obj.orientation.z]
        rospy.loginfo("Humans position received: %r" %self.humans)

    def distance(self, p1, p2):
        return math.sqrt(math.pow(p2[0] - p1[0], 2) + math.pow(p2[1] - p1[1], 2))

    def localizer_cb(self, data):
        self.map_robot_pose[0] = data.pose.pose.position.x
        self.map_robot_pose[1] = data.pose.pose.position.y
        o = data.pose.pose.orientation
        q = (o.x, o.y, o.z, o.w)
        euler = tf.transformations.euler_from_quaternion(q)
        self.map_robot_pose[2] = euler[2] # yaw

    def send_rotation_goal(self,angle):
        rospy.loginfo("Sending goal..")

        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        yaw = angle + self.map_robot_pose[2]
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)

        # AS ALTERNATIVE, JUST ROTATE MARRTINO TOWARDS EACH PEOPLE AND USE THE PERSON
        # DETECTION MODULE
        goal.target_pose.pose.position.x = self.map_robot_pose[0]
        goal.target_pose.pose.position.y = self.map_robot_pose[1]
        goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        rospy.loginfo("ANGLE OF PERSON RECEIVED (DEGREE): %f\n" %(angle * 180 / math.pi))
        rospy.loginfo("ANGLE OF PERSON RECEIVED (RADIANS): %f\n" %angle)
        rospy.loginfo("MARRTINO ANGLE (DEGREE): %f\n" % (self.map_robot_pose[2] * 180 / math.pi))
        rospy.loginfo("MARRTINO ANGLE (RADIANS): %f\n" %self.map_robot_pose[2])
        rospy.loginfo("ANGLE SENT AS GOAL (DEGREE): %f\n" % (yaw * 180 / math.pi))
        rospy.loginfo("ANGLE SENT AS GOAL (RADIANS): %f\n" %yaw)

        self.ac_movebase.send_goal(goal)

        rospy.loginfo("move_base: rotational goal %r sent!" % (goal))

    def send_goal(self, target_pose):
        rospy.loginfo("Sending goal..")

        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Consider a circle as origin the person and a radius of a TARGET_RADIUS m.
        # Marrtino does not want to go too close to the person.
        offset_x = TARGET_RADIUS if (target_pose[0] < 0) else - TARGET_RADIUS
        offset_y = TARGET_RADIUS if (target_pose[1] < 0) else - TARGET_RADIUS

        target_x =  self.map_robot_pose[0] + target_pose[0] + offset_x
        target_y =  self.map_robot_pose[1] + target_pose[1] + offset_y

        # Yaw of each person is already in degree
        yaw = target_pose[2] + self.map_robot_pose[2]
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)

        # AS ALTERNATIVE, JUST ROTATE MARRTINO TOWARDS EACH PEOPLE AND USE THE PERSON
        # DETECTION MODULE
        goal.target_pose.pose.position.x = target_x
        goal.target_pose.pose.position.y = target_y
        goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])


        rospy.loginfo("Robot pose: %r\n" %([self.map_robot_pose[0],self.map_robot_pose[1]]))
        rospy.loginfo("Human pose (w.r.t MARRTINO): %r\n" %([target_pose[0],target_pose[1]]))
        rospy.loginfo("DISTANCE BETWEEN TARGET POSITION AND HUMAN: %r\n" % (
        self.distance([self.map_robot_pose[0] + target_pose[0], self.map_robot_pose[1] + target_pose[1]],
                          [target_x, target_y])))
        rospy.loginfo("PERSON RECEIVED (w.r.t MARRTINO) ANGLE: %f\n" % (target_pose[2] * 180 / math.pi))
        rospy.loginfo("MARRTINO ANGLE (DEGREE): %f\n" % (self.map_robot_pose[2] * 180 / math.pi))
        rospy.loginfo("ANGLE (DEGREE) SENT AS GOAL: %f\n" % (yaw * 180 / math.pi))

        self.ac_movebase.send_goal(goal)

        rospy.loginfo("move_base: goal %r sent!" % (goal))



    def action_thread(self, params):
        rospy.loginfo("Leg detection action thread started")
        rospy.sleep(2)

        self.loc_sub = rospy.Subscriber(BASE_POSE_GROUND_TRUTH_TOPIC, Odometry, self.localizer_cb)
        rospy.sleep(0.5)

        #for index,key in self.humans.items():
        self.send_goal(self.humans[0])
            #self.send_rotation_goal(key[2])

        finished = False
        while self.do_run and not finished:
            self.ac_movebase.wait_for_result(rospy.Duration(1))
            status = self.ac_movebase.get_state()  # 1 ACTIVE, 3 SUCCEEDED, 4 ABORTED
            finished = (status == GoalStatus.SUCCEEDED) or (status == GoalStatus.ABORTED)

        state = self.ac_movebase.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("move_base: goal succeeded!")
        else:
            rospy.loginfo("move_base: goal failed!")

        self.ac_movebase.get_result()
        self.ac_movebase.cancel_all_goals()
        self.loc_sub.unregister()


if __name__ == "__main__":

    params = None
    if (len(sys.argv) > 1):
        params = sys.argv[1]

    a = GotoPersonDetectedActionProxy(ACTION_NAME)

    if params is not None:
        a.execute(params)  # blocking, CTRL-C to interrupt
    else:
        a.run_server()  # blocking, CTRL-C to interrupt