import sys, os, socket

import rospy
from std_msgs.msg import String

# sys.path.append("....")                  # <--- rospy PLEXI folder
from fluentproxy import FluentProxy

FLUENT_NAME = 'mypersonhere'  # <--- fluent name

ROS_NODE_NAME = 'takephoto'
TOPIC_takephoto = '/takephoto'
PARAM_takephoto_image_folder = '%s/imagefolder' % ROS_NODE_NAME

SERVER = 'localhost'
PD_PORT = 9252



class MyPersonHereFluentProxy(FluentProxy):  # <--- fluent class

    def __init__(self, fluentname, rosnode=True):
        FluentProxy.__init__(self, fluentname, rosnode)
        self.takephoto_pub = rospy.Publisher(TOPIC_takephoto, String, queue_size=1)
        rospy.sleep(0.1)

        self.server = SERVER
        self.pd_port = PD_PORT


    def __del__(self):
        FluentProxy.__del__(self)

    def setServer(self,server, port):
        rospy.param_get()
        self.server = server
        self.port = port

    def sensingStep(self):
        rospy.loginfo("MyPersonHere sensing step started")
        s_pd = String()
        s_pd.data = 'send %s %s' %(self.server, self.pd_port)

        self.takephoto_pub.publish(s_pd)
        rospy.sleep(0.2)

        if (self.server != 'localhost') and (self.server != '127.0.0.1'):
            rospy.sleep(1)


        ####################
        # Person Detection #
        ####################
        try:
            sock = socket.socket()
            sock.connect((self.server, self.pd_port))
            sock.sendall("GETRESULT\n\r")
            data = sock.recv(256)
            data = data.strip().decode('UTF-8')
            print(data)
            sock.close()
        except:
            data = None

        howManyPeopleDetected = 0
        if data is None:
            value = -1
        else:
            # Data from PD comes in this form:
            ### [howManyPeopleDetected confidence[0] confidence[1]..confidence[N]] ###
            v = data.split(' ')

            # If data is 'none' something broke up and we set to -1 (not false nor true)
            if v[0] == 'none':
                value = -1
            else:

                try:
                    howManyPeopleDetected = int(v[0])
                except ValueError:
                    print("\nThe string cannot be casted to a float.\n")

                value = 0

                # If at least one person is detected with confidence above 60%, then we set the fluent to 1, otherwise we set it to 0
                if howManyPeopleDetected == 0:
                    value = 0
                else:
                    for i in range(howManyPeopleDetected):
                        if (float(v[i+1]) > 0.6):
                            value = 1


        self.setValue(value)  # 1: true,  0: false,  -1: unknown
        if value == 1:
            rospy.loginfo("\nSomeone is here!")
            rospy.loginfo("\nPeople detected: %d" % howManyPeopleDetected)
            for i in range(howManyPeopleDetected):
                rospy.loginfo("\nConfidence of person_%d: %f" % (i + 1, float(v[i + 1])))
            rospy.loginfo("\nFluent mypersonhere set to value: %d" % value)
            rospy.loginfo("\n----------------------\n")

        else:
            rospy.loginfo("\nNobody is here!")
            rospy.loginfo("\nPeople detected: %d" % howManyPeopleDetected)
            rospy.loginfo("\nFluent mypersonhere set to value: %d\n----------------------\n" % value)

        rospy.loginfo("MyPersonHere sensing step started")

    def fluent_thread(self, params):
        v = params.split('_')
        while self.do_run:
            self.sensingStep()
            rospy.sleep(5)


if __name__ == "__main__":

    params = ''
    if (len(sys.argv) > 1):
        params = sys.argv[1]

    t = MyPersonHereFluentProxy(FLUENT_NAME)  # <--- fluent class
    t.execute(params)  # blocking, CTRL-C to interrupt