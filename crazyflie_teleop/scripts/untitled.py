#!/usr/bin/env python

import threading
import rospy
from std_msgs.msg import String

class Test():
        def __init__(self):
                self.pub = rospy.Publisher("/test",String,queue_size=10)
                self.i = 0
                self.msg = "hello world!"+str(self.i)

                timer = threading.Thread(target=self.timing)
                printer = threading.Thread(target=self.helloWorld)
                self.lock = threading.Lock()
                timer.start()
                printer.start()

        def timing(self):
                while 1:
                        self.lock.acquire()
                        self.msg = "hello world!"+str(self.i)
                        self.pub.publish(self.msg)
                        self.lock.release()

        def helloWorld(self):
                while 1:
                        self.lock.acquire()
                        self.i+=1
                        if self.i > 100:
                                self.i = 0
                        self.lock.release()

if __name__ == "__main__":
        
        rospy.init_node("test")
        test = Test()
        rospy.spin()
