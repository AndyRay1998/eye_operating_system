# Author Info
Unit: Beihang University, Beijing, China
Name: Changyi Lei
Email: 1998andyray@gmail.com
Github URL: https://github.com/AndyRay1998


# Environment
ubuntu 16.04 64bits
python 3.7.0 (default)
python 2.7.17 (for ROS)
ROS-kinetic
numpy 1.15.1


# Scripting Languages
This workspace include both .cpp and .py executable files for flexibility and robustness, 
and the default language is c++ because of its high effectiveness.


# Usage of roslaunch
NOTE that at present galil ,hyperion and YAMAHA (serial port) provide c++ support.
Use  $roslaunch galil_mixed eye_op_robot.launch                    to evoke .cpp executable files
and  $roslaunch galil_mixed eye_op_robot.launch file_suffix:=.py   to evoke .py executable files
For more args, see the .launch file in "eye_op_common/launch".


# Python Import Tips
In file "galil_overall_listener.py", we "import gclib_python.example". 
This syntax is possible only if there is a "__init__.py" file in folder "gclib_python". 
The same applies to "from . import gclib" in "example.py".


# Version Control
Add "add_definitions(-std=c++11)" to the first line of CMakeLists.txt for compilation. It is version control for c++.
All .py files add #!usr/bin/env python3 for version control.


# File Structure
'omni_packages' is not a ROS package, but those four folders inside it are. 'omni_packages' is just for better file structure.


# About API
Omni phantom API is inside folder 'omni_packages'. See 'omni_packages/README.md' for installation guidance.
OpenHaptic SDK is in 'omni_packages/OpenHapticsAE_Linux_v3_0' or can be downloaded here: https://github.com/fsuarez6/phantom_omni/releases
Try $./env.sh under the path of "~/eye_op_robot_mixed" for automatic API installation first if you are using 64-bit system.
Hyperion API is in 'hyperion_mixed/script', which you do not need to install manually.
