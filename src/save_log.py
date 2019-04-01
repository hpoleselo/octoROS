#!/usr/bin/env python
import rospy
from octo_ros.msg import PrinterState

class FileWriter(object):
    def __init__(self):
        rospy.init_node("DataRetriever")
        self.sub = rospy.Subscriber("/printer3d", PrinterState, self.storeData)
        #elf.f = open(printerlog.txt, "a+")
        #clean

    def storeData(self, msg):
        """ Callback function to store all the received data in a txt file. """
        wholeMsg = msg
        wholeMsg = str(wholeMsg)
        #TODO add something to clean the current file
        f = open("printerlog.txt","a+")
        f.write(wholeMsg)

        # Space between the messages
        f.write("\n")
        f.write("%%% \n")
        #f.close()


def run():
    FileWriter()
    rospy.spin()

if __name__ == "__main__":
    run()

