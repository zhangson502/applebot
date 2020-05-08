#Python basic libraries
sudo apt-get install python-pip
pip install opencv-python
pip install requests
pip install urllib

#install ROS navigation packages
sudo apt-get install ros-kinetic-navigation

#ROS Dynamic reconfiguration
sudo apt-get install ros-kinetic-dynamic-reconfigure
sudo apt-get install ros-kinetic-ddynamic-reconfigure

#Install and configure D435 driver
echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' | sudo tee /etc/apt/sources.list.d/realsense-public.list
sudo apt-key adv --keyserver keys.gnupg.net --recv-key 6F3EFCDE
sudo apt-get update
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg



