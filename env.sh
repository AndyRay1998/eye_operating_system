#!/bin/bash

source devel/setup.bash

echo "adding executable files"
chmod +x ./src/eye_op_common/script/common_listener.py
chmod +x ./src/eye_op_common/script/common_talker.py
chmod +x ./src/eye_op_common/script/UI.py

chmod +x ./src/eye_robotics_2/script/joint_state_pub.py

chmod +x ./src/galil_mixed/script/galil_overall_listener.py

chmod +x ./src/hyperion_mixed/script/hyperion_talker.py

chmod +x ./src/yamaha_mixed/script/yamaha_listener.py

echo "installing OMNI API"

sudo apt-get install --no-install-recommends freeglut3-dev g++ libdrm-dev \
libexpat1-dev libncurses5-dev libraw1394-dev libx11-dev libxdamage-dev    \
libxext-dev libxt-dev libxxf86vm-dev tcsh unzip x11proto-dri2-dev         \
x11proto-gl-dev x11proto-print-dev

sudo dpkg -i ./src/omni_packages/OpenHapticsAE_Linux_v3_0/PHANTOM Device Drivers/64-bit/phantomdevicedrivers_4.3-3_amd64.deb

sudo dpkg -i ./src/omni_packages/OpenHapticsAE_Linux_v3_0/OpenHaptics-AE 3.0/64-bit/openhaptics-ae_3.0-2_amd64.deb

sudo ln -s /usr/lib/x86_64-linux-gnu/libraw1394.so.11 /usr/lib/libraw1394.so.8

sudo ln -s /usr/lib64/libPHANToMIO.so.4.3 /usr/lib/libPHANToMIO.so.4

sudo ln -s /usr/lib64/libHD.so.3.0.0 /usr/lib/libHD.so.3.0

sudo ln -s /usr/lib64/libHL.so.3.0.0 /usr/lib/libHL.so.3.0

sudo rm -f /usr/lib/libraw1394.so.8

sudo ln -s /usr/lib/x86_64-linux-gnu/libraw1394.so.11 /usr/lib/libraw1394.so.8

catkin_make

echo "The API should have been installed successfully if no errors occurred!"
echo "But still you must check 'http://fsuarez6.github.io/projects/geomagic-touch-in-ros/' for more details!"
