# OctoROS
This projects aims to be a bridge between 3D Printers and ROS. 
It uses OctoPrint to control the printer, and get the info that is sent to ROS topics. 


## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

## Prerequisites

First you'll need to install OctoPrint, the instructions are available at https://github.com/foosel/OctoPrint. But the easiest way that I see and in order to integrate the application with ROS is with the following instructions:

1. Checkout OctoPrint:  ```$ git clone https://github.com/foosel/OctoPrint.git```
2. Change into the OctoPrint folder: ```$ cd OctoPrint```
3. Request pip to run the setup file, i.e install all the necessary dependencies: ```$ sudo pip2.7 install .```
4. After installing OctoPrint, you should run it using: ```$ octoprint serve```   
5. Now you should verify your installation opening a web browser and going to http://localhost:5000
6. If everything went right you should see the OctoPrint home screen  

PS: In case you encountered a conflict error with PyYAML on step ```3.``` make sure you remove the previous installation from PyYAML from your computer by doing: 

 ```$ sudo rm -rf /usr/local/lib/python2.7/dist-packages/yaml ```

And then calling the setup file again to install all the dependencies but ignoring PyYAML (this is necessary because PyYAML is usually not completely deleted.)

 ```$ sudo pip2.7 install . --ignore-installed ${PyYAML} ``` 

**_By entering the OctoPrint home screen for the first time you should setup your 3D Printer using the wizard from Octoprint, make sure to setup it correctly._** Don't forget to include the baudrate from your 3D Printer and enable the API Key **(respectively copying it)**.

**Paste the copied API Key on the 13th line of** ``` messenger.py ```

In case the printer you're using is a MakerBot, then you need to install the GPX plug-in, because OctoPrint doesn't support the x3g format, only gcode. GPX will take care of the conversion from gcode to x3g. Do the following:
1. In the octoPrint home screen go to configurations/Plugin manager/Get More
2. Search for GPX and click install 
3. After GPX is installed, make sure to check in the plugin's list if the GPX plugin is enabled
4. Make sure to configure correctly in the GPX plugin all the settings for your 3D printer (machine, gcode flavor and other settings)

PS: In case you encountered the error that the octoprint could not open your port, do the following:

 ```$ sudo chmod 666 /dev/ttyACM0 ``` 

## Installing

This project should be run from source, to do so just go to your ROS workspace (supposing is called ``` catkin_ws ```):

``` $ cd ~/catkin_ws/src ```

And clone it with: 

``` $ git clone https://github.com/hpoleselo/octoROS.git```

Tether octoROS with your catkin workspace:

``` $ cd ~/catkin_ws ```

``` $ catkin_make ```

Make ``` octoROS.py ``` executable so ROS can identify it:

``` $ roscd octo_ros/src ```

``` $ chmod +x octoROS.py ```

## Usage
In order to use it, you need to have your octoprint server running, have a model uploaded to it, **changing the file name in the 16th line to the uploaded file name** and then run the launch file: 

``` $ roslaunch octo_ros connect_to_printer.launch ```

The output should be like this:
``` 
[INFO] [1553712847.912730]: Initialized!
[INFO] [1553712847.920685]: Starting to print model testfile.gcode
[INFO] [1553712847.928079]: Started retrieving data from 3D Printer. Hear to the topic if you want to see the streamed data.
```

Go to another terminal and

``` $ rostopic echo /printer3d ```

The output should be like this:
```
ricky@c-137:~/wspace_ros/src/octo_ros/src$ rostopic echo /printer3d
timestamp: 23
date_time: "2019-04-17 18:10:10.608935"
temp_tool1_actual: 32.0
temp_tool2_actual: 32.0
temp_bed_actual: 87.0
file_size: 126475
file_name: "testfile.gcode"
printer3d_state: "Bed heating"
progress: 1.43190348148
time_left: 912.0
time_elapsed: 220.0
temp_tool1_goal: 0.0
temp_tool2_goal: 0.0
temp_bed_goal: 110.0
---
```

So it will start printing the model and outputting the progress and some printer measurements to the ```/printer3d``` ROS topic until the printing is finished. When the printing is done, a flag will be set, i.e a boolean will be sent to ```printer3d/finishedPrinting```.



## Authors

* **Daniel Mascarenhas** - *Initial work* - [ielson](https://github.com/ielson)
* **Henrique Poleselo** - *Additional functionalities* - [hpoleselo](https://github.com/hpoleselo)

See also the list of [contributors](https://github.com/ielson/octoROS/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

Many thanks to the octoprint team, that made this awesome software
