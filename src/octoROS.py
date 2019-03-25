#!/usr/bin/env python

""" octoROS - integrating octoprint with ROS, so it's possible to control your 3d printer from your robot
This library is based in the octoprint api, more info can be found here:
http://docs.octoprint.org/en/master/api/index.html
"""

import sys

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String

import messenger

fileToPrint = 'FlexiRexColor1.gcode'

class RosInterface(object):
    def __init__(self):
        self.print_pub = rospy.Publisher('printer3d', String, queue_size=100)
        self.printFinished_pub = rospy.Publisher('printer3d/finishedPrinting', Bool, queue_size=10)
        self.rate = rospy.Rate(0.1)  # 0.1 hz
        print("initialized")

    def performConnection(self):
        self.print_pub.publish("Trying to connect to the 3D Printer...")
        connection = messenger.connectToPrinter()
        if connection.status_code != 204:
            # TODO change all exceptions to the right ones
            raise Exception('Could not connect to printer, error code: {}'.format(connection.status_code))
        self.print_pub.publish("Connection succeeded")

    def printAndGetStatus(self, modelName):
        printing = messenger.printModel(modelName)
        if printing.status_code != 204:
            pass
            # raise Exception('Could not print, status code: {}'.format(printing.status_code))
        self.print_pub.publish("Starting to print model {}".format(modelName))
        progress = messenger.progressTracking()
        rospy.loginfo("Started retrieving data from 3D Printer. Hear to the topic if you want to see the streamed data.")
        while progress < 100 and not rospy.is_shutdown():
            # Retrieving all data
            tool0TempA, tool1TempA, bedTempA, tool0TempT, tool1TempT, bedTempT, isPrinting, isPaused = messenger.getprinterInfo()
            fileName, estimatedTime = messenger.getFileInfo()
            progress = messenger.progressTracking()
            timeLapse = messenger.getTimeLapse()

            # TODO change to a action
            
            self.print_pub.publish("--- Actual data ---")
            # TODO Need to see the type returned by getPrinterInfo, so I can make a msg and send it to the right topic
            self.print_pub.publish("File name: {}, Printing: {}, Paused:{}".format(fileName, isPrinting, isPaused))
            self.print_pub.publish("Progress: {}%".format(progress))
            self.print_pub.publish("Estimated time left: {}h".format(estimatedTime))
            self.print_pub.publish(
                "Tool 0 temp: {}C, tool 1 temp: {}C, bed temp: {}C".format(tool0TempA, tool1TempA, bedTempA))
            self.print_pub.publish("--- Target data ---")
            self.print_pub.publish(
                "Tool 0 temp: {}C, tool 1 temp: {}C, bed temp: {}C".format(tool0TempT, tool1TempT, bedTempT))
            
            self.rate.sleep()
        print("Successful print!")
        self.print_pub.publish("Successful print")
        self.printFinished_pub.publish(True)


def main(args):
    # ROS was not catching interrupt exceptions, so I had to disable signals and use the KeyboardInterrupt exception
    rospy.init_node('printerWatcher', anonymous=True, disable_signals=True)
    interf = RosInterface()
    interf.printAndGetStatus(fileToPrint)


if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
        messenger.cancelPrinting()

