#!/usr/bin/env python

""" octoROS - integrating octoprint with ROS, so it's possible to control your 3d printer from your robot
This library is based in the octoprint api, more info can be found here:
http://docs.octoprint.org/en/master/api/index.html
"""

import sys
import datetime
import rospy
from std_msgs.msg import Bool, String
from octo_ros.msg import PrinterState

import messenger

counter = 0
fileToPrint = 'testfile.gcode'

class RosInterface(object):
    def __init__(self):
        self.print_pub = rospy.Publisher('printer3d', PrinterState, queue_size=100)
        self.printFinished_pub = rospy.Publisher('printer3d/finishedPrinting', Bool, queue_size=10)
        self.rate = rospy.Rate(0.1)  # 0.1 hz
        rospy.loginfo("---- OctoROS Initialized! ----")

    def performConnection(self):
        rospy.loginfo("Trying to connect to the 3D Printer...")
        connection = messenger.connectToPrinter()
        if connection.status_code != 204:
            # TODO change all exceptions to the right ones
            raise Exception('Could not connect to printer, error code: {}'.format(connection.status_code))
        self.print_pub.publish("Connection succeeded")

    def getDateTime(self):
        date_time = datetime.datetime.now()
        # Convert it to string to insert in our message
        date_time = str(date_time)
        return date_time

    def countTimeStamp(self):
        """ Currently implementing our own time stamp to count the messages that are being sent.
        TODO: implement it using Header()"""
        global counter
        #print "dentro do timestamp", counter
        counter += 1
        return int(counter)

    def countTime(self, ts):
        global counter
        print "dentro do timer", counter
        if counter < 0:
            timeInSeconds = ts*10
            print timeInSeconds
            return timeInSeconds

    def printPartAndGetStatus(self, modelName):
        printing = messenger.printModel(modelName)
        if printing.status_code != 204:
            pass
            raise Exception('Could not print, status code: {}'.format(printing.status_code))
        rospy.loginfo("Starting to print model {}".format(modelName))
        
        # Temporary, but disregarding some variables
        progress, _, _, _, _ = messenger.printingProgressTracking()
        rospy.loginfo("Started retrieving data from 3D Printer. Hear to the topic if you want to see the streamed data.")
        while progress < 100 and not rospy.is_shutdown():
            progress, printingTimeLeft, timeElapsed, fileName, fileSize = messenger.printingProgressTracking()
            # Retrieving all data
            tool0TempA, tool1TempA, bedTempA, tool0TempT, tool1TempT, bedTempT, state = messenger.getprinterInfo()
            # mudar nome
            date_time = self.getDateTime()
            ts = self.countTimeStamp()
            #timeElapsed = self.countTime(ts)
            


            # If the printer is not printing something then it returns None
            if progress == None:
                progress = 0.0
            
            # Encapsulate all the data
            pstate = PrinterState()
            pstate.timestamp = ts
            pstate.date_time = date_time
            pstate.temp_tool1_actual = tool0TempA
            pstate.temp_tool2_actual = tool1TempA
            pstate.temp_bed_actual = bedTempA
            pstate.file_name = fileName
            pstate.file_size = fileSize
            pstate.printer3d_state = state
            pstate.progress = progress
            #pstate.time_elapsed = timeElapsed
            pstate.time_left = printingTimeLeft
            pstate.temp_tool1_goal = tool0TempT
            pstate.temp_tool2_goal = tool1TempT
            pstate.temp_bed_goal = bedTempT
            self.print_pub.publish(pstate)
            self.rate.sleep()
        self.print_pub.publish("Successful print")
        self.printFinished_pub.publish(True)


def main(args):
    # ROS was not catching interrupt exceptions, so I had to disable signals and use the KeyboardInterrupt exception
    rospy.init_node('printerWatcher', anonymous=True, disable_signals=True)
    interf = RosInterface()
    interf.printPartAndGetStatus(fileToPrint)


if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down and cancelling printing...")
        messenger.cancelPrinting()
