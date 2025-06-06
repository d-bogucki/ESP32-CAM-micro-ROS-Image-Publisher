# ESP32-CAM Image Publisher micro-ROS Node

This directory contains a simple micro-ROS node publishing a jpeg image over the DDS network
, in particular it uses the ROS sensor_msgs/msg/CompressedImage message.
The compressed image is taken from the OV2640 Camera-Chip inside the ESP32-CAM board.

## Installation and Setup
Note: It is assumed that the reader already installed the ESP-IDF framework for working with ESP MCU targets, and knows the basic command either with the CLI or for example the ESP-IDF VScode extension.
In order to use/change the application clone it inside a folder, then do the following steps:
- Select Flash Method, Port and Target (e.g., ESP32 CHIP, since ESP32-CAM is not an official Espressif target)
- inside the /components directory clone the micro-ros_espidf_component, for the ROS2 iron irwini distro for example run the following command:
  git clone -b iron https://github.com/micro-ROS/micro_ros_espidf_component.git
- clean the micro-ros component through the "idf.py clean-microros" command
- run the "idf.py reconfigure" command, so all the XRCE-DDS changes written in the app-colcon.meta file will be updated correctly
- change the Wi-Fi settings using your SSID and password via the sdkconfig file or the "idf.py menuconfig" tool
- After the first "idf.py reconfigure" a directory "managed_components" should be present in your project directory, containing the camera driver. Now change the DMA buffer size in the ll_cam.c file of the camera driver, at line 471 reducing it to 1280 for example (from default value of 2048)
- run "idf.py reconfigure" and then finally build the app with "idf.py build" command
- connect the board to the selected Port (e.g., /dev/tty/USB0) and Flash the application


More detailed instructions coming soon.. DB
