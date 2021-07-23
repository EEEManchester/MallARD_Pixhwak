# MallARD_Pixhwak
This repository is about controlling the thrusters in MallARD via MAVROS using Pixhawk4. Two tasks needs to be completed. First is to compile the custom firmware for Pixhawk4 named Audusub and flash it onto Pixhawk 4. The second is installation of MAVROS and MAVLINK. The two submodules in this repository contain source files of the custom Ardusub firmware and MAVROS.

# Required hardware
* Pixhawk 4 flight controller
* PC running ununtu 18.04 and ros melodic
* Joypad (ps4)
* USB cables
* Thrusters, speed controllers and batteries for testing

## 1. Flash pixhawk 4 with custom firmware - Autopilot_MALLARD
ArduPilot MALLARD (AP-M) is a customised ArduPilot firmware for MALLARD. It is built based on the current stable ArduSub release (ArduSub-4.0.3).

AP-M provides a *Custom* frame configuration adapted to the thruster allocation used on MALLARD. This frame configuration allows higher level motion command input such as *move_forward*, *turn_left*, etc. AP-M also provides two new frame configurations, i.e. *Joystick PWM Control* and *ROS PWM Control*, which enables sending a PWM signal directly to each individual motor by pushing a joystick or publishing a ROS topic, respectively. The *ROS PWM Control* frame assumes thruster allocation is dealt with within ROS.

### Hardware
For this build guide, we use a Pixhawk 4 and direct USB connection to a PC running Ubuntu.

### Build
#### Install git
In case you have not yet installed *git*, run the following commands in terminal:
```
sudo apt update
sudo apt install git
```

### Clone repository
Open a terminal and `cd` to our desired root folder for the repository, then clone the main [ArduPilot MALLARD](https://github.com/EEEManchester/ArduPilot_MALLARD) repository using your preferred authentication protocol.

* HTTPS:
```
git clone --recursive https://github.com/EEEManchester/ArduPilot_MALLARD.git
```
* SSH:
```
git clone --recursive git@github.com:EEEManchester/ArduPilot_MALLARD.git
```

Make sure you log into the correct GitHub account that has access to [EEEManchester](https://github.com/EEEManchester).

### Setup build and compile environment
[A script](https://github.com/ArduPilot/ardupilot/blob/master/Tools/environment_install/install-prereqs-ubuntu.sh) is provided to automatically setup your build environment. It will install the tool chain in addition it will also install the math proxy ground station controller and the neccessary elements to do software in the loop simulation on your PC of the code.
```
cd ArduPilot_MALLARD
Tools/environment_install/install-prereqs-ubuntu.sh -y
```
> **Note:** If the end of the command promp is not showing something similar to `echo xxx end------`, the setup is unsuccessful. If it complains about `'some-python-package' has no installation candidate`, your system is probably configured for Python3. You need to run the follow instead:
> ```
> Tools/environment_install/install-prereqs-ubuntu-py3.sh -y
> ```

Reload the path (log-out and log-in to make permanent) to store the new paths to all those tools:
```
. ~/.profile
```

### Build with WAF
**Make sure you are in ardupilot folder.**

**Note:** For Python3 users, you need to replace `waf` with `waf-py3`.

If you have previously built the firmware, you may want to clean WAF first:
```
./waf clean
```

Then
```
./waf configure --board Pixhawk4
./waf sub
```

After the code finishes, you can find the new firmware `ardusub.apj` in the `build/Pixhawk4/bin` directory.

### Upload
#### Using WAF
There are two conditions for this to work:
1. with a direct USB connection to the Pixhawk
2. only after configuring and building with `waf` before
```
./waf --upload sub
```
#### Using GCS
* [Mission Planner](https://ardupilot.org/planner/docs/common-loading-chibios-firmware-onto-pixhawk.html#uploading-as-custom-firmware)
* [QGroundControl](https://docs.qgroundcontrol.com/master/en/SetupView/Firmware.html#loading-firmware)

---
#### Original guides
In ~~unlikely~~ situation that the above instructions do not work, you may want to check out the original ArduPilot guides:

* [Developers on ArduSub Wiki](http://www.ardusub.com/developers/developers.html)
* [Setting up the Build Environment (Linux/Ubuntu)](https://ardupilot.org/dev/docs/building-setup-linux.html)
* [BUILD.md](https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md)

### TEST the custom firmware using QGC 

1.Install QGroundControl
QGroundControl can be installed/run on Ubuntu LTS 18.04 (and later).

Ubuntu comes with a serial modem manager that interferes with any robotics related use of a serial port (or USB serial). Before installing QGroundControl you should remove the modem manager and grant yourself permissions to access the serial port. You also need to install GStreamer in order to support video streaming.

Before installing QGroundControl for the first time:

1. On the command prompt enter:
    Add the user's name to the dialout group, not the user root even though the command is run as root:   
    ```
    sudo usermod -a -G dialout $USER
    ```  
    Remove the modem manager and grant yourself permissions to access the serial port:  
    ``` 
    sudo apt-get remove modemmanager -y
    ```  
    Install GStreamer in order to support video streaming (probably not be use in MallARD):  
    ```
    sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
    ```

2. Logout and login again to enable the change to user permissions.

To install QGroundControl:  
  * Download QCG from this link [QGroundControl.AppImage](https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage).  
  * Install (and run) using the terminal commands from the folder containing the .AppImage file:
    ```
    chmod +x ./QGroundControl.AppImage  
    ./QGroundControl.AppImage
    ```
    
 When QGC launches you may see an error saying 'Parameters are missing from firmware. You may be running a version of firmware which is not fully supported .....'. This is normal, just proceed.

3. Finish the sensors setup in QGroundControl - click sensors and follow procedures for accelerometer and compass - autopilot rotation set to none

2.Hardware setup for test  
Connect the hardware as the photo shown below:
![hardware setup](https://user-images.githubusercontent.com/77399327/126214195-7c65f4f2-6351-4708-9f5e-b52a294ba59a.jpg)

3.Test the motor  
* Launch QGC, using command 
```
./QGroundControl.AppImage 
```
Frame selection (For more details about thruster allocation, please check this link [README_math.md](https://github.com/EEEManchester/MallARD_Pixhwak/blob/main/README_math.md)  
ArduSub frame can be configured by setting [FRAME_CONFIG](https://www.ardusub.com/developers/full-parameter-list.html#frameconfig-frame-configuration). In addition to the built-in options, we offer two additional configurations. The Custom frame has also been modified to reflect the thruster allocation of MALLARD 003.

* Set the thrust allocation:  
Click Q![Screenshot from 2021-07-20 17-06-53](https://user-images.githubusercontent.com/77399327/126358068-e0ca4cc3-65eb-4550-873f-3a1ce5251b17.png) icon --> Vehicle Setup --> Parameters.  Using the search bar: FRAME_CONFIG. Set it to 8 in advanced settings (the number associated with the joystick PWM control thrust allocation).
You may see its parameter editing panel:  
![Screenshot from 2021-07-21 20-35-48](https://user-images.githubusercontent.com/77399327/126549189-31f30050-e76f-4249-88ec-97d6426c9de2.png)


* Make sure the motors you want to test are enabled:     
 In parameters setting, using the search bar: SERVO1_FUNCTION. Check it has been set to motor1 (sometimes displayed as 33). Do the same steps to SERVO2_FUNCTION (Motor2 or 34), SERVO3_FUNCTION (Motor3 or 35), SERVO4_FUNCTION (motor4 or 36). You may see the panel like this.
![Screenshot from 2021-07-21 10-34-34](https://user-images.githubusercontent.com/77399327/126467201-8a5fb0f8-61a5-49fb-982b-ad57c3400842.png)

* Make sure you system ID is 255:  
In Parameters, using search bar: SYSID_MYGCS, make sure it has been set to 255 (Advanced settings).  
![Screenshot from 2021-07-21 12-07-26](https://user-images.githubusercontent.com/77399327/126479324-263afdb9-82ff-41af-8063-a9e6af0a489d.png) 
* Add a virtual joystick:  
Click Q icon -->  Application settings --> General --> tick virtual joystick ![Screenshot from 2021-07-20 17-09-21](https://user-images.githubusercontent.com/77399327/126358419-3b18b2d9-4661-4400-aa69-0b49365d3181.png)    
* Arm and using the virtual joystick to make the thruster move:  
Use JOYSTICK_PWM_CONTROL frame and when you put input on x thruster 1 should move but no others. input on y only thruster 2 moves . input on yaw only thruster 3 moves. input on z only thruster 4 moves.



## 2. Control thrusters via ROS using MAVROS and MAVLINK - 
### Source installation  

Use `wstool` utility for retrieving sources and  [catkin tools](https://catkin-tools.readthedocs.io/en/latest/) for build.


**NOTE:** The source installation instructions are for the ROS Melodic release.

1. Install catkin_tool and dependencies: 
``` 
sudo apt-get install python-catkin-tools python-rosinstall-generator -y
```  
2. For Noetic use:  
```
sudo apt install python3-catkin-tools python3-rosinstall-generator python3-osrf-pycommon -y
```

3. Create the workspace: unneeded if you already has workspace

```
mkdir -p ~/catkin_ws/src  
cd ~/catkin_ws  
catkin init  
wstool init src  
```
4. Install MAVLink

    we use the Melodic reference for all ROS distros as it's not distro-specific and up to date
```
rosinstall_generator --rosdistro melodic mavlink | tee /tmp/mavros.rosinstall`
```
5. Install MAVROS: get source (upstream - released)  
```
rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
```

6. Create workspace & deps

```
wstool merge -t src /tmp/mavros.rosinstall
``` 
```
gedit ~/catkin_ws/src/.rosinstall
```  
change   
'- git:  
    local-name: mavros  
    uri: https://github.com/mavlink/mavros.git  
    version: 1.8.0'  
to  
'- git:  
    local-name: mavros_mallard  
    uri: https://github.com/EEEManchester/mavros_mallard.git  
    version: dev'  
```
wstool update -t src -j4
```  
```
rosdep install --from-paths src --ignore-src -y
```

7. Install GeographicLib datasets:  
```
sudo ./src/mavros_mallard/mavros/scripts/install_geographiclib_datasets.sh
```

8. Build source   
```
catkin build
```

9. Make sure that you use setup.bash or setup.zsh from workspace. Else rosrun can't find nodes from this workspace.
```
source devel/setup.bash
```

10. Make the control node executable.
```
cd src/mavros_mallard/joy2thr/src
chmod +x joyControl.py
```
### QGC configuration
Before use MAVROS to drive MallARD, some parameters need to be set in QGC. open QGC

* Make sure the motors are disabled in QGC:  
Click Q icon, and click Vehicle Setup, click parameters, using search bar: SERVO1_FUNCTION, set it to 0 (Disabled). Do the same to SERVO2_FUNCTION, SERVO3_FUNCTION, SERVO4_FUNCTION. This step can make all thrusters controlled by the signal from MAVROS.  
![Screenshot from 2021-07-21 10-17-10](https://user-images.githubusercontent.com/77399327/126464753-f748aeb7-415a-4fc8-a33b-feb2629fd9e6.png)

* In Parameters setting, using search bar: SYSID_MYGCS, set SYSID_MYGCS = 1 .This set can make sure the GCS talks to MAVROS.  
![Screenshot from 2021-07-20 17-34-36](https://user-images.githubusercontent.com/77399327/126361966-96e3e88f-519c-4faa-ba2a-d746de810c40.png)

* Switch the frame from test mode to MallARD thruster allocation mode. In Parameters, using search bar: FRAME_CONFIG. Set it to 7.  
![Screenshot from 2021-07-21 12-30-58](https://user-images.githubusercontent.com/77399327/126482055-ba55b727-eb82-48bd-bcf4-d08ef16f6c52.png)
* Back to QGC desktop. Click Q icon, and click Application Setting, click AutoConnection to following devices, just tick the UDP and distick the rest as the picture shown below:  
![tick UPD](https://user-images.githubusercontent.com/77399327/126353159-63572722-cf02-4400-9b32-6df1c6168384.png)  

### Connect PS4 joystick to PC via Bluebooth
 1. Press and hold the central PS Button and the Share button for three seconds until the lightbar at the top of the controller begins to flash. Next open up the Bluetooth settings on your PC then select 'Wireless Controller'. You can also connect the joystick via USB. 
 2. Configuring and Using a Linux-Supported Joystick with ROS:
    * Start by installing the package:   
    ```
    sudo apt-get install ros-melodic-joy
    ```
    * Configuring the Joystick:
    Connect your joystick to your computer. Now let's see if Linux recognized your joystick.  
    ```
    ls /dev/input/
    ```  
    You will see a listing of all of your input devices similar to below:
    ```
    by-id    event0  event2  event4  event6  event8  mouse0  mouse2  uinput
    by-path  event1  event3  event5  event7  js0     mice    mouse1  
    ```
    As you can see above, the joystick devices are referred to by jsX ; in this case, our joystick is js0. Let's make sure that the joystick is working. 
    ```
    sudo jstest /dev/input/jsX
    ```
    You will see the output of the joystick on the screen. Move the joystick around to see the data change. 
    ```
    Driver version is 2.1.0.
    Joystick (Logitech Logitech Cordless RumblePad 2) has 6 axes (X, Y, Z, Rz, Hat0X, Hat0Y)
    and 12 buttons (BtnX, BtnY, BtnZ, BtnTL, BtnTR, BtnTL2, BtnTR2, BtnSelect, BtnStart, BtnMode, BtnThumbL, BtnThumbR).
    Testing ... (interrupt to exit)
    Axes:  0:     0  1:     0  2:     0  3:     0  4:     0  5:     0 Buttons:  0:off  1:off  2:off  3:off  4:off  5:off  6:off  7:off  8:off  9:off 10:off 11:off
    ```

3. Joystick setup in Open QGC.   
For the first time using joystick, it needs to be set. Open the QGC. In the Vehicle setup, find the joystick on the right bar and complete the setup step by step according to the instruction. This is the panel you may find. 
![from iOS](https://user-images.githubusercontent.com/77399327/126820961-72e60c37-9eea-46d9-9c2f-86904befb8c6.jpg)
### Control MallARD via joystick via ROS
* Connect the pixhawk 4 to the PC and connect MallARD's ESCs to the pixhawk 4. Power the ESCs and connect ESCs to thrusters.

* Close QGC

* Launch mavros nodes in shell#1:  
```
roslaunch mavors apm.launch
```
* Open the QGC

* Launch control node(including joy node) in Shell#2:  
```
roslaunch joy2thr joy2thr.launch
```  
* Arm the vehicle by command in Shell#3:  
```
rosservice call /mavros/cmd/arming "value: true"
``` 
* Disarm:
```
rosservice call /mavros/cmd/arming "value: false"
```


### Original guides
1. [README](https://github.com/EEEManchester/mavros_mallard/blob/master/README_MAVROS.md)
2. [Installation instructions](https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation)
3. [ROS Wiki](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick)
