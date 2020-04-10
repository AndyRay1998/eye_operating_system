# Construction and Implementation of Software Control Platform for Ophthalmic Operation Robot Based on ROS  
This project tries to develop a flexible and modular platform for researches into ophthalmic operation robot based on ROS.  
At present, it only provides a good software frame but not wholistic and systematic functions from input to output.  
  
## Getting Started
The following content will help you deploy this project to your PC smoothly.  
Please read carefully.  
  
## Author Info
Unit: Beihang University, Beijing, China  
Name: Changyi Lei  
Email1: 1998andyray@gmail.com  
Email2: andyray1998@qq.com  
Github URL: https://github.com/AndyRay1998  
  
## Test Environment  
ubuntu 16.04 64bits  
python 3.7.0 (default)  
python 2.7.17 (for ROS)  
ROS-kinetic  
numpy 1.15.1  
  
## Installation
1. Create a folder named `eye_op_robot_mixed` or whatever you prefer  
2. Download and extract all files into `eye_op_robot_mixed`  
3. Source and compile the workspace:  
```Bash  
$ cd eye_op_robot_mixed  
$ source devel/setup.bash  
$ catkin_make  
```  
   Test ROS communication  
```Bash  
$ roslaunch eye_op_common eye_op_robot.launch  
```  
4. Try auto installation of Omni Phantom API if you are using 64-bit system  
```Bash  
$ ./env.sh  
```  
* 4.1 If any errors happen, see inside `omni_pacakges/README.md` for manual installation guidance.  
* OpenHaptic SDK is in `omni_packages/OpenHapticsAE_Linux_v3_0` or can be downloaded here https://github.com/fsuarez6/phantom_omni/releases .  

5. Hyperion API is in `hyperion_mixed/src`, which you do not need to install manually.  
6. Test Omni API  
```Bash  
$ roslaunch omni_common omni.launch  
```  
7. After that, you still need to see http://fsuarez6.github.io/projects/geomagic-touch-in-ros/ for more information.  
  
## Scripting Languages  
This workspace include both .cpp and .py executable files for flexibility and robustness.  
The default language is c++ because of its high effectiveness.  
NOTE that at present galil ,hyperion and YAMAHA (serial port) provide c++ support.  
  
## Usage of roslaunch - Entrance of Project
There are five parameters designed :  
* `yamaha_address, yamaha_freq, yamaha_timeout` for YAMAHA serial communication;  
* `hyperion_address` for Hyperion device connection;  
* `file_suffix` for programming language selection.  
All of them have a default value.  
e.g.  
Evoke .cpp executable files:  
```Bash  
$ roslaunch galil_mixed eye_op_robot.launch  
```  
Evoke .py executable files:  
```Bash  
$ roslaunch galil_mixed eye_op_robot.launch file_suffix:=.py  
```  
Check "eye_op_common/launch/eye_op_robot.launch" for source code.  
  
## Python Import Tips  
In file `galil_overall_listener.py`, we `import gclib_python.example`.  
This syntax is possible only if there is a `__init__.py` file in folder `gclib_python`.  
The same applies to `from . import gclib` in `example.py`.  
  
## Version Control  
Add `add_definitions(-std=c++11)` to the first line of `CMakeLists.txt` for compilation. It is version control for c++.  
All .py files add `#!usr/bin/env python3` for version control.  
  
# File Structure
There are five folders inside `src`.  
`eye_op_common` is for shared scripts, including foward and inverse kinematic and jacobian calculation scripts.  
`hyperion_mixed` is for Hyperion device. API locates in ```src``` and executable scripts are in `script`.  
`galil_mixed` is for Galil motion control card. API locates in ```src``` and executable scripts are in `script`.  
`yamaha_mixed` is for Galil motion control card. API locates in ```src``` and executable scripts are in `script`.  
`omni_packages` is not a ROS package itself, but those four folders inside it are. `omni_packages` is just for better file structure. And those four packages inside are Omni API that you should have installed following previous instruction.  

## License
This project is released under MIT license.  
See LICENSE.md for further information.  

