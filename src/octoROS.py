#!/usr/bin/env python

""" octoROS - integrating octoprint with ROS, so it's possible to control your 3D printer from your robot
This library is based in the Octoprint API, more info can be found here:
http://docs.octoprint.org/en/master/api/index.html
"""

import os
import sys
import datetime
import rospy
from std_msgs.msg import Bool, String
from octo_ros.msg import PrinterState
import messenger

counter = 0

class RosInterface(object):
    def __init__(self):
        self.print_pub = rospy.Publisher('printer3d', PrinterState, queue_size=100)
        self.printFinished_pub = rospy.Publisher('printer3d/finishedPrinting', Bool, queue_size=10)
        self.rate = rospy.Rate(0.1)  # 0.1 hz
        rospy.loginfo("---- OctoROS Initialized! ----")
        rospy.loginfo("---- Searching for requested file ----")
        self.fileToPrint = self.getFile()
        self.fileToPrint = self.searchFile(self.fileToPrint)
        if self.fileToPrint != -1:
            rospy.loginfo("---- Sending requested file to OctoPrint ----")
            self.printPartAndGetStatus(self.fileToPrint)
        else:
            rospy.logwarn("Could not send the file to the printer, shutting down...")

    def getFile(self):
        try:
            self.fileToPrint = rospy.get_param('/printerWatcher/file_name')
            return self.fileToPrint
        except(NameError):
            rospy.logerr("Didn't receive the print file.") 

    def searchFile(self, fileToPrint):
        files = os.listdir(".")
        gcode_files = []
        for file in files:
            if file == self.fileToPrint:
                return self.fileToPrint
            elif file.endswith('.gcode'):
                gcode_files.append(file)
        if file != self.fileToPrint:
            notFoundString = "Requested file: %s not found in the package /src directory" %self.fileToPrint
            rospy.logwarn(notFoundString)
            rospy.logwarn("Files with the extension .gcode that have been found: ")
            for i in range(len(gcode_files)):
                print gcode_files[i]
            # -1 is a flag for not founding any file with the given name.
            return -1

    def getDateTime(self):
        """ Get the actual date and time. """
        date_time = datetime.datetime.now()
        date_time = str(date_time)
        return date_time

    def countTimeStamp(self):
        """ Currently implementing our own time stamp to count the messages that are being sent.
        TODO: implement it using Header() and seq"""
        global counter
        counter += 1
        return int(counter)

    def countTime(self, ts):
        """ Takes the timestamp and 'converts' it to count the time since the printer has started to print.  """
        global counter
        if counter >= 2:
            # Transform to the actual time
            ts = ts - 1
            timeInSeconds = ts*10
            return int(timeInSeconds)

    def printPartAndGetStatus(self, fileToPrint):
        """ Sends command to print the wished part and sends all the data retrieved from the printer to ROS """
        try:
            printing = messenger.printModel(fileToPrint)
        except(AttributeError):
                sys.exit()
                rospy.loginfo("Printer not connected to the OctoPrint server.")
                rospy.is_shutdown()
        bootString = "Starting to print model %s" %fileToPrint 
        rospy.loginfo(bootString) 

        # Temporary, but disregarding some variables
        progress, _, _, _ = messenger.printingProgressTracking()
        rospy.loginfo("Started retrieving data from 3D Printer. Hear to the topic if you want to see the streamed data.")
        while progress < 100 and not rospy.is_shutdown():
            # Update progress and get all the remaining data
            progress, printingTimeLeft, fileName, fileSize = messenger.printingProgressTracking()
    
            
            try:
                # Retrieving all data
                bedTempA, bedTempT, tool0TempA, tool0TempT, state, tool1TempA, tool1TempT = messenger.getprinterInfo()
                date_time = self.getDateTime()
                ts = self.countTimeStamp()
                timeElapsed = self.countTime(ts)
                if timeElapsed == None:
                    timeElapsed = 0

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
                pstate.time_elapsed = timeElapsed
                pstate.time_left = printingTimeLeft
                pstate.temp_tool1_goal = tool0TempT
                pstate.temp_tool2_goal = tool1TempT
                pstate.temp_bed_goal = bedTempT
                self.print_pub.publish(pstate)
                self.rate.sleep()
            except(ValueError,UnboundLocalError):
                rospy.logerr("Not connected to the printer, check if the USB cable is connected and if the port is enabled.")
                sys.exit()

        rospy.loginfo("Sucessful printing.")
        self.printFinished_pub.publish(True)


def main(args):
    # ROS was not catching interrupt exceptions, so I had to disable signals and use the KeyboardInterrupt exception
    rospy.init_node('printerWatcher', anonymous=True, disable_signals=True)
    interf = RosInterface()


if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down and cancelling printing...")
        messenger.cancelPrinting()
