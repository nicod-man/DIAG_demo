# template action

import sys, os, socket

import rospy
from std_msgs.msg import String

from actionproxy import ActionProxy

ACTION_NAME = 'welcome'
TOPIC_STAGESAY = '/stage_say'
TOPIC_TAKEPHOTO = '/takephoto'


EMOTIONS_LIST = ["Angry", "Disgust", "Fear", "Happy", "Neutral", "Sad", "Surprise"]

FER_SERVER = 'localhost'
FER_PORT = 9253


class WelcomeActionProxy(ActionProxy):

    def __init__(self, actionname):
        ActionProxy.__init__(self, actionname)
        self.stagesay_pub = rospy.Publisher(TOPIC_STAGESAY, String, queue_size=1, latch=True)
        rospy.sleep(0.2)
        self.takephoto_pub = rospy.Publisher(TOPIC_TAKEPHOTO, String, queue_size=1)
        rospy.sleep(0.2)

        self.server = FER_SERVER
        self.fer_port = FER_PORT

    def __del__(self):
        ActionProxy.__del__(self)

    def action_thread(self, params):
        rospy.loginfo("Welcome thread started")
        s = String()
        s.data = 'send %s %s' %(self.server, self.fer_port)
        self.takephoto_pub.publish(s)
        rospy.sleep(1)

        if (self.server != 'localhost') and (self.server != '127.0.0.1'):
            rospy.sleep(1)

        try:
            sock = socket.socket()
            sock.connect((self.server, self.fer_port))
            sock.sendall("GETRESULT\n\r")
            data = sock.recv(256)
            data = data.strip().decode('UTF-8')
            print(data)
            sock.close()
        except Exception as e:
            print("Impossible to estabilish a connection with %s:%d. Exception is %s" %(self.server,self.fer_port,e))
            data = None


        welcome = "Hi! I'm MARRtino. Nice to meet you. "
        # Data comes as [emotion confidence]
        if data is not None:

            v = data.split(' ')
            if (v[0]== 'error'):
                pass
            else:
                emotion = v[0]
                emotion_confidence = float(v[1])
                toPrint = '[emotion: ' + emotion + ', emotion_confidence: ' + str(emotion_confidence) + ']'
                rospy.loginfo(toPrint)
                if (emotion_confidence > 0.6):
                    # "Angry", "Disgust", "Fear", "Happy", "Neutral", "Sad", "Surprise"
                    if (emotion == "Angry"):
                        emoted = "It seems from your face that you are angry. There's something wrong with me?"
                    elif (emotion == "Happy"):
                        emoted = "It seems from your face that you are happy. I'm glad to see it."
                    elif (emotion == "Sad"):
                        emoted = "It seems from your face that you are sad. I'm sorry. Can I help you to make it better?"
                    elif (emotion == "Disgust"):
                        emoted = "It seems from your face that you are disgusted. What is disgusting you?"
                    elif (emotion == "Fear"):
                        emoted = "It seems from your face that you are scared. What is scaring you?"
                    elif (emotion == "Surprise"):
                        emoted = "It seems from your face that you are surprised. Is there any good news?"
                    else:
                        emoted = ''
                    welcome = welcome + emoted

        s = String()
        s.data = welcome
        self.stagesay_pub.publish(s)
        cmd = 'echo "TTS[en] %s" | netcat -w 1 localhost 9001' %welcome
        os.system(cmd)

        t = 2.0 / 10.0
        dt = 0.25
        while self.do_run and t > 0:
            rospy.sleep(dt)
            t -= dt
        s.data = ""
        self.stagesay_pub.publish(s)
        rospy.sleep(dt)

        rospy.loginfo("Welcome thread end")
if __name__ == "__main__":

    params = None
    if (len(sys.argv) > 1):
        params = sys.argv[1]

    a = WelcomeActionProxy(ACTION_NAME)

    if params is not None:
        a.execute(params)  # blocking, CTRL-C to interrupt
    else:
        a.run_server()  # blocking, CTRL-C to interrupt

