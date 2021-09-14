# template for fluent implementation
import tf
import math,time, sys
import rospy

from std_msgs.msg import String
from sensor_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry

from fluentproxy import FluentProxy

FLUENT_NAME = 'legdetection'

LEG_DETECTOR_TOPIC = '/edge_leg_detector'
BASE_POSE_GROUND_TRUTH_TOPIC = '/base_pose_ground_truth'
TOPIC_amcl_pose = '/amcl_pose'
LEG_FLUENT_TOPIC = '/human_positions'

LEG_FLUENT_NODE='leg_fluent'

PEOPLE_THRESHOLD = 2

# Since the coordinates of the people are given w.r.t. the origin of the robot,
# computing atan2(y,x) returns the orientation of the person w.r.t the current
# orientation of the robot

class LegDetectorFluentProxy(FluentProxy):

    def __init__(self, fluentname, rosnode=True):
        FluentProxy.__init__(self, fluentname, rosnode)
        rospy.sleep(0.1)

        self.fluent_leg_pub = rospy.Publisher(LEG_FLUENT_TOPIC, PoseArray, queue_size=10)
        rospy.sleep(0.1)

        rospy.Subscriber(LEG_DETECTOR_TOPIC, PoseArray, self.legdetection_cb)
        rospy.sleep(0.1)

        # rospy.Subscriber(BASE_POSE_GROUND_TRUTH_TOPIC, Odometry, self.marrtino_position_cb)
        # rospy.sleep(0.1)
        self.other_position = []


    def __del__(self):
        FluentProxy.__del__(self)


    def legdetection_cb(self, data):
        self.other_position = data.poses
        rospy.loginfo(data.poses)

    def sensingStep(self):
        rospy.loginfo("LegDetection sensing step started")
        value = 0

        ##### This sleep is needed since self.other_position is not populated #####
        rospy.sleep(2)

        if ( len(self.other_position) > 0):
            # Compute distances between marrtino and each people.
            distances = list()

            # We need to publish a PoseArray with the orientation of the people detected
            msg = PoseArray()

            msg.header.frame_id = "map_leg"
            msg.header.stamp = rospy.Time.now()

            for i in self.other_position:

                # L1- distance from marrtino
                position = math.sqrt(math.pow(i.position.x, 2) + math.pow(i.position.y, 2))
                print("Distance from marrtino: %f" %position)

                # If the position is less than a threshold we consider that as a person and not as a wall
                if position <= PEOPLE_THRESHOLD:
                    distances.append(position)

                    # We need first to populate each Pose and then append them to the PoseArray
                    person_xyz = Point(i.position.x,i.position.y,0)
                    person_orientation = Quaternion(0,0,math.atan2(i.position.y,i.position.x),0)
                    person_pose = Pose(person_xyz,person_orientation)
                    msg.poses.append(person_pose)

            # The value of the fluent is True iff someone has been considered as a person
            if (len(distances) > 0):
                value = 1
                self.fluent_leg_pub.publish(msg)
                rospy.sleep(0.5)
        # print(msg)
        self.setValue(value)
        rospy.loginfo("Fluent value set to: %d" %self.getValue())
        rospy.loginfo("LegDetection sensing step endend")

    def fluent_thread(self, params):
        #v = params.split('_')
        while self.do_run:
            self.sensingStep()
            rospy.sleep(5)

if __name__ == '__main__':

    params = ''
    if (len(sys.argv) > 1):
        params = sys.argv[1]

    t = LegDetectorFluentProxy(FLUENT_NAME)
    t.execute(params)



