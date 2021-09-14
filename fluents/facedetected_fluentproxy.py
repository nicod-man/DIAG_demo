import sys, os, socket

import rospy
from std_msgs.msg import String

# sys.path.append("....")                  # <--- rospy PLEXI folder
from fluentproxy import FluentProxy

FLUENT_NAME = 'facedetection'  # <--- fluent name
TOPIC_takephoto = '/takephoto'

SERVER = 'localhost'
FD_PORT = 9989

class FaceDetectionFluentProxy(FluentProxy):  # <--- fluent class

    def __init__(self, fluentname, rosnode=True):
        FluentProxy.__init__(self, fluentname, rosnode)
        self.takephoto_pub = rospy.Publisher(TOPIC_takephoto, String, queue_size=1)
        rospy.sleep(0.1)

        self.server = SERVER
        self.fd_port = FD_PORT


    def __del__(self):
        FluentProxy.__del__(self)

    def setServer(self,server, port):
        rospy.param_get()
        self.server = server
        self.port = port

    def sensingStep(self):
        rospy.loginfo("FaceDetection sensing step started")
        s_fd = String()
        s_fd.data = 'send %s %s' % (self.server, self.fd_port)
        self.takephoto_pub.publish(s_fd)

        rospy.sleep(0.2)

        if (self.server != 'localhost') and (self.server != '127.0.0.1'):
            rospy.sleep(1)


        ####################
        #  Face Detection  #
        ####################
        try:
            sock = socket.socket()
            sock.connect((self.server, self.fd_port))
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
            # Data from FD comes as an int [number_of_people_detected]

            v = data.split(' ')

            # If data is 'none' something broke up and we set to -1 (not false nor true)
            if v[0] == -1:
                value = -1
            else:
                try:
                    howManyPeopleDetected = int(v[0])
                    # If at least one person is detected with confidence above 60%, then we set the fluent to 1, otherwise we set it to 0
                    if howManyPeopleDetected == 0:
                        value = 0
                    else:
                        value = 1
                except ValueError:
                    print("\nThe string cannot be casted to a float.\n")

        self.setValue(value)  # 1: true,  0: false,  -1: unknown
        if value == 1:
            rospy.loginfo("\nSomeone is here!")
            rospy.loginfo("\nPeople detected: %d" % howManyPeopleDetected)

            rospy.loginfo("\nFluent FaceDetected set to value: %d" % value)
            rospy.loginfo("\n----------------------\n")

        else:
            rospy.loginfo("\nNobody is here!")
            rospy.loginfo("\nPeople detected: %d" % howManyPeopleDetected)
            rospy.loginfo("\nFluent FaceDetected set to value: %d\n----------------------\n" % value)

        rospy.loginfo("FaceDetection sensing step end")
    def fluent_thread(self, params):
        v = params.split('_')
        while self.do_run:
            self.sensingStep()
            rospy.sleep(5)


if __name__ == "__main__":

    params = ''
    if (len(sys.argv) > 1):
        params = sys.argv[1]

    t = FaceDetectionFluentProxy(FLUENT_NAME)  # <--- fluent class
    t.execute(params)  # blocking, CTRL-C to interrupt