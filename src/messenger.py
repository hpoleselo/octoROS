#!/usr/bin/env python2.7

""" octoROS - integrating octoprint with ROS, so it's possible to control your 3d printer from your robot
This library is based in the octoprint api, more info can be found here:
http://docs.octoprint.org/en/master/api/index.html
"""

import requests

octoIP = "http://127.0.0.1"
octoPort = ":5000"
apiKey = "63A30C9DD2D44F239C1ED9CA68963EA9"

# json key with the API Key
standardHeader = {'X-Api-Key': apiKey}

# Second extruder checker.
secondExtruderExists = None

# Connection handling
def connectToPrinter():
    connectionData = {"command": "connect", "port": "/dev/ttyACM0", "baudrate": 115200, "printerProfile": "_default",
                      "save": True, "autoconnect": True}
    return requests.post(_url("connection"), json=connectionData, headers=standardHeader, timeout=5)

# --- SENDING COMMANDS TO THE PRINTER / JOB OPERATIONS ---
# --- FUNCTIONS TO SEND TO THE PRINTER --- 

def printModel(modelName):
    printData = {'command': 'select', 'print': True}
    url = _url('files/local/{}'.format(modelName))
    return requests.post(url, json=printData, headers=standardHeader, timeout=5)

def cancelPrinting():
    jsonData = {'command':'cancel'}
    return requests.post(_url('job'), headers=standardHeader, timeout=5, json=jsonData)

def pausePrinting():
    jsonData = {'command':'pause', 'action':'pause'}
    return requests.post(_url('job'), headers=standardHeader, timeout=5, json=jsonData)

def resumePrinting():
    jsonData = {'command':'pause', 'action':'resume'}
    return requests.post(_url('job'), headers=standardHeader, timeout=5, json=jsonData)

def checkFilesSD():
    """ Check all the files on the Octoprint Server and on the SD Card. """
    files = []
    response = requests.get(_url('files'), headers=standardHeader, timeout=5)
    numberOfFiles = len(response.json()['files'])

    for i in range(0,numberOfFiles):
        try:
            files.append(response.json()['files'][i]['name'])
        except(IndexError):
            print ("The list has been iterated.")

def modelSelection():
    pass

# --- RETRIEVING INFORMATION FROM THE PRINTING PROCESS ---
# --- FUNCTIONS TO SEND TO THE PRINTER --- 

def printingProgressTracking():
    response = requests.get(_url('job'), headers=standardHeader, timeout=5)
    progress = response.json()['progress']['completion']
    # In seconds! 
    printTimeLeft = response.json()['progress']['printTimeLeft']
    fileName = response.json()['job']['file']['name']
    fileName = str(fileName)
    # In bytes
    fileSize = response.json()['job']['file']['size']    
    return progress, printTimeLeft, fileName, fileSize


def rateState(isBedHeating, isToolHeating, isPrinting, isPaused, isReadyToPrint, isCancelled):
    """ Function that rates all the states from the printer and returns just one state to send xas a String
    to ROS. Short: encapsulates all states in one function. This function is expandable, you can add more 
    states to be rated."""

    if isPaused:
        finalState = "Printing process has been paused"
    elif isBedHeating:
        finalState = "Bed heating"
    elif isToolHeating:
        finalState = "Extruder heating"
    elif isPrinting:
        finalState = "Printing"
    elif isCancelled:
        finalState = "Cancelling printing"
    elif isReadyToPrint:
        finalState = "Available and ready for Printing"
    else:
        finalState = "Offline"
    #elif:
    #    finalState = "Busy_Yellow"
    #elif:
    #    finalState = "Busy_Red"
    return finalState


def checkToolHeating(tool0TempA, tool0TempT):
    """ Functions to check if the extruder is heating. This will be sent to rateState function to rate the atual state of the printer. """
    isToolHeating = False
    diff2 = tool0TempT - tool0TempA
    #print "Temperature difference from Tool:", diff2
    if diff2 > 0.0:
        isToolHeating = True
    elif diff2 <= 0.0:
        isToolHeating = False
    else:
        isToolHeating = False
    return isToolHeating


def checkBedHeating(bedTempA, bedTempT):
    """ Functions to check if the bed is heating. This will be sent to rateState function to rate the atual state of the printer. """
    isBedHeating = False
    diff1 = bedTempT - bedTempA
    #print "Temperature difference from Bed:", diff1
    if diff1 > 0.0:
        isBedHeating = True
    elif diff1 <= 0.0:
        isBedHeating = False
    else:
        isBedHeating = False
    return isBedHeating

def checkTool1Availability():
    """ Checks if there is more than one tool. If there's only one extruder, it returns 0 to both variables tool1TempA and tool1TempT. """
    response = requests.get(_url('printer'), headers=standardHeader, timeout=5)
    try:
        tool1TempA = response.json()['temperature']['tool1']['actual']
        tool1TempT = response.json()['temperature']['tool1']['target']
        isFound = True
    except(KeyError):
        tool1TempA = 0
        tool1TempT = 0
        isFound = False
    return tool1TempA, tool1TempT, isFound

def getprinterInfo():
    response = requests.get(_url('printer'), headers=standardHeader, timeout=5)
    global secondExtruderExists

    # Actual measurements
    tool0TempA = response.json()['temperature']['tool0']['actual']
    bedTempA = response.json()['temperature']['bed']['actual']

    # Additional information besides what Daniel has made
    isPrinting = response.json()['state']['flags']['printing']
    isPaused = response.json()['state']['flags']['pausing']
    isReadyToPrint = response.json()['state']['flags']['operational']
    isCancelled = response.json()['state']['flags']['cancelling']
    
    # Targets
    tool0TempT = response.json()['temperature']['tool0']['target']
    bedTempT = response.json()['temperature']['bed']['target']

    # Checks for a second tool, if available gets the data from it
    if (secondExtruderExists == None):
        tool1TempA, tool1TempT, isFound = checkTool1Availability()
        print ("Found a second tool.")
        # Updates the state of secondExtruderExists so it doesn't check anymore
        secondExtruderExists = isFound
    elif (secondExtruderExists):
        tool1TempA = response.json()['temperature']['tool1']['actual']
        tool1TempT = response.json()['temperature']['tool1']['target']


    # Call functions to encapsulate all states in one
    isToolHeating = checkToolHeating(tool0TempA, tool0TempT)
    isBedHeating = checkBedHeating(bedTempA, bedTempT)
    state = rateState(isBedHeating, isToolHeating, isPrinting, isPaused, isReadyToPrint, isCancelled)
    return tool0TempA, tool1TempA, bedTempA, tool0TempT, tool1TempT, bedTempT, state

def _url(path):
    """ Function to pass the URL """
    octoAddress = octoIP + octoPort + '/api/'
    return octoAddress + path
