# OctoROS
This projects aims to be a bridge between 3D Printers and ROS. 
It uses OctoPrint to control the printer, and get the info that is sent to ROS topics. 


## Tested Environment

- Makerbot Replicator 2X (Firmware 7.6), Ultimaker 2+ and GtMax3D
- Ubuntu LTS 16.04.06 (Xenial Xerus)
- ROS Kinetic   
- Python 2.7
- Octoprint 1.3.11

## Prerequisites

Make sure you have Python 2.7 along with pip installed, if not so:

```$ wget https://bootstrap.pypa.io/get-pip.py```

```$ sudo python2.7 get-pip.py```

Since OctoROS relies on the ```requests``` package, it's better to install it in forehand:

```$ sudo pip2.7 install requests```

Now we can install OctoPrint, the detailed instructions are available at https://github.com/foosel/OctoPrint. But the easiest way that I see and in order to integrate the application with ROS is with the following instructions:

1. Checkout OctoPrint:  ```$ git clone https://github.com/foosel/OctoPrint.git```
2. Change into the OctoPrint folder: ```$ cd OctoPrint```
3. Request pip to run the setup file, i.e install all the necessary dependencies: ```$ sudo pip2.7 install .``` or ``` $  sudo pip2 install . ```
4. After installing OctoPrint, you should run it using: ```$ octoprint serve```   
5. Now you should verify your installation opening a web browser and going to http://localhost:5000
6. If everything went right you should see the OctoPrint home screen  

PS: In case you encountered a conflict error with PyYAML on step ```3.``` make sure you remove the previous installation from PyYAML from your computer by doing: 

 ```$ sudo rm -rf /usr/local/lib/python2.7/dist-packages/yaml ```

 and

```$ sudo rm -rf /usr/lib/python2.7/dist-packages/yaml ```

And then calling the setup file again to install all the dependencies but ignoring PyYAML (this is necessary because PyYAML is usually not completely deleted.) **Note that this would work for any other package from Python that is conflicting with the installation, i.e we could replace PyYAML for the package causing you trouble.**

 ```$ sudo pip2.7 install . --ignore-installed ${PyYAML} ``` 

**_By entering the OctoPrint home screen for the first time you should setup your 3D Printer using the wizard from Octoprint, make sure to setup it correctly._** Don't forget to include the baudrate from your 3D Printer and enable the API Key **(respectively copying it)**.

**Paste the copied API Key on the 13th line of** ``` messenger.py ```

In case the printer you're using is a MakerBot, then you need to install the GPX plug-in, because OctoPrint doesn't support the x3g format, only gcode. GPX will take care of the conversion from gcode to x3g. Do the following:
1. In the octoPrint home screen go to configurations/Plugin manager/Get More
2. Search for GPX and click install 
3. After GPX is installed, make sure to check in the plugin's list if the GPX plugin is enabled
4. Make sure to configure correctly in the GPX plugin all the settings for your 3D printer (machine, gcode flavor and other settings)

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

Set the command to PATH so we can easily execute ``` print_file.sh``` from any directory:

``` $ cd ~/ ```

``` $ echo PATH=$PATH:~/catkin_ws/src/octoROS/src >> .bashrc  ```

Add small function to open the port and run the OctoPrint server at once:

``` $ cd ~/ ```

``` $ echo "run_octoprint() { sudo chmod +666 /dev/ttyACM0 ; octoprint serve ; }" >> .bashrc  ```

## Usage
In order to use OctoROS, you need to have your Octoprint server running, have the wished model uploaded to it and located on ``` /src ``` folder from OctoROS:

First run the Octoprint server along with opening the port for the printer (MAKE SURE THE PRINTER IS CONNECTED TO THE COMPUTER!)

``` $ run_octoprint ```

Open up another terminal and send the model you want to print with the following command:

``` $ print_file.sh YOURMODEL.gcode ```

The output should be like this:
``` 
[INFO] [1563900881.544640]: ---- OctoROS Initialized! ----
[INFO] [1563900881.545309]: ---- Searching for requested file ----
[INFO] [1563900881.548957]: ---- Sending requested file to OctoPrint ------
[INFO] [1563900881.594716]: Starting to print model..
[INFO] [1563900881.608451]: Started retrieving data from 3D Printer. Hear to the topic if you want to see the streamed data.
[INFO]: Checking for a second extruder...
```

Go to another terminal and

``` $ rostopic echo /printer3d ```

The output should be like this:
```
ricky@c-137:~$ rostopic echo /printer3d
timestamp: 5
date_time: "2019-07-23 13:55:21.690592"
temp_tool1_actual: 25.0
temp_tool2_actual: 26.0
temp_bed_actual: 28.0
file_size: 1052938
file_name: "part2Rep2x.gcode"
printer3d_state: "Bed heating"
progress: 0.169240728021
time_left: 1059
time_elapsed: 40
temp_tool1_goal: 0.0
temp_tool2_goal: 0.0
temp_bed_goal: 110.0
---
```

So it will start printing the model and outputting the progress and some printer measurements to the ```/printer3d``` ROS topic until the printing is finished. When the printing is done, a flag will be set, i.e a boolean will be sent to ```printer3d/finishedPrinting```.

## Example (Makerbot) [Deprecated]
A testing file ``` testfile.gcode ``` for the Makerbot Replicator 2X is provided in order to execute this example. The file is located on the folder ```src```.

1. ```Connect the printer to your computer via USB cable.```

Supposing that the installation steps have been followed correctly we open up a new terminal and:

2.  ```$ octoprint serve```

**Now we should see the screen of the octoprint on your browser by acessing the adress ```http://localhost:5000/``` , you should see the following screen**:
![Screenshot](/screenshots/pic1.png) 

3. ``` Upload the gcode file you want to print, in our case, testfile.gcode to the Octoprint Web Interface in the following section: ```

![Screenshot](/screenshots/pic2.png) 

Open up a new terminal and:

4. ``` $ roslaunch octo_ros connect_to_printer.launch ```

Since the gcode is on the same folder as the script that interfaces Octoprint and in the script ``` octoros.py ``` on the line 17th the name of the file
to be printed matches the actual file, then it's going to send the file automatically when we execute the command in step ```4```.

You should see something like this:

![Screenshot](/screenshots/pic3.png) 

If we wish to check the stats of the printer, we open up a new terminal and give the following command:

5. ``` $ rostopic echo /printer3d ```

What you will see is the stats like the picture below:

![Screenshot](/screenshots/pic4.png) 

## Authors

* **Daniel Mascarenhas** - *Initial work* - [ielson](https://github.com/ielson)
* **Henrique Poleselo** - *Additional functionalities* - [hpoleselo](https://github.com/hpoleselo)

See also the list of [contributors](https://github.com/ielson/octoROS/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

Many thanks to the octoprint team, that made this awesome software
