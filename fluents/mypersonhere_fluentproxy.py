# template for fluent implementation

import sys, os, socket

import rospy
from std_msgs.msg import String

# sys.path.append("....")                  # <--- rospy PLEXI folder
from fluentproxy import FluentProxy

FLUENT_NAME = 'mypersonhere'  # <--- fluent name

# takephoto topics/params
ROS_NODE_NAME = 'takephoto'
TOPIC_takephoto = '/takephoto'
PARAM_takephoto_image_folder = '%s/imagefolder' % ROS_NODE_NAME

SERVER = 'localhost'
SPD_PORT = 9250
FD_PORT = 9251
PD_PORT = 9252
FER_PORT = 9253


class MyPersonHereFluentProxy(FluentProxy):  # <--- fluent class

    def __init__(self, fluentname, rosnode=False):
        FluentProxy.__init__(self, fluentname, rosnode)
        self.takephoto_pub = rospy.Publisher(TOPIC_takephoto, String, queue_size=1)
        rospy.sleep(0.1)

        self.server = SERVER

        # Every module accepts connection on its own port
        self.spd_port = SPD_PORT
        self.fd_port = FD_PORT
        self.pd_port = PD_PORT
        self.fer_port = FER_PORT

    def __del__(self):
        FluentProxy.__del__(self)

    def setServer(self,server, port):
        rospy.param_get()
        self.server = server
        self.port = port

    def sensingStep(self):

        sockOK = False
        # connecting to stagepersondetection
        # connecting to person_detection
        # connecting to face_detection
        # connecting to face_expression_recognition

        try:
            # sock_fd = socket.socket()
            # sock_fd.connect((self.server, self.fd_port))
            # sock_fd.close()

            sock_pd = socket.socket()
            sock_pd.connect((self.server, self.pd_port))
            sock_pd.close()

            # sock_fer = socket.socket()
            # sock_fer.connect((self.server, self.fer_port))
            # sock_fer.close()

            sockOK = True
        except Exception as e:
            print(e)
            print("Cannot connect to server %s:%d" % (self.server, self.pd_port))
            self.server = '192.168.0.205'  # try this other server

        if not sockOK:
            try:
                sock = socket.socket()
                sock.connect((self.server, self.pd_port))
                sock.close()
                sockOK = True
            except Exception as e:
                print(e)
                print("Cannot connect to server %s:%d" %(self.server, self.pd_port))
                return

        # s_fd = String()
        # s_fd.data = 'send %s %s' % (self.server, self.fd_port)

        s_pd = String()
        s_pd.data = 'send %s %s' %(self.server, self.pd_port)

        # s_fer = String()
        # s_fer.data = 'send %s %s' % (self.server, self.fer_port)


        # self.takephoto_pub.publish(s_fd)
        self.takephoto_pub.publish(s_pd)
        # self.takephoto_pub.publish(s_fer)

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

        if data is None:
            value = -1

        else:
            # Data from PD comes in this form:
            # [howManyPeopleDetected confidence[0] confidence[1]..confidence[N]]
            v = data.split(' ')
            for i in range(len(v)):
                print("\n v[%d]: %f" %(i,float(v[i])))

            if v[0] == 'none':
                self.setValue(-1)
                return

            try:
                howManyPeopleDetected = int(v[0])
            except ValueError:
                print("\nThe string cannot be casted to a float.\n")

            value = 0
            if howManyPeopleDetected >= 1:
                for i in range(howManyPeopleDetected):
                    if (float(v[i+1]) > 0.6):
                        value = 1

        self.setValue(value)  # 1: true,  0: false,  -1: unknown
        rospy.set_param("/pnp/conditionsBuffer/mypersonhere", 1)
        print("\n value: %d" %value)
        print("\nself.getValue(): %d\n----------------------\n" %self.getValue())

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