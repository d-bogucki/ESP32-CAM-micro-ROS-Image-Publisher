# ESP32-CAM with OpenCV test:

This directory contains time computation analysis of some OpenCV functions inside an ESP project.
In particular the functions "cvtColor()", "threshold()", "findContours()", and "approxPolyDP()" are analyzed.
Note: The application still uses the micro-ROS framework to transmit the binary image over the
ROS2 network for visual purposes, hence remember to start the micro-ros agent on the MPU side. 


## OpenCV pre-compiled Library integration:
- Follow the steps described in the README file of the following Github repository [OpenCV for ESP32](https://github.com/joachimBurket/esp32-opencv/tree/master)
In particular clone the build-release pre-compiled library inside the /opencv directory in the project's main directory.

- Remember to enable the partition table modification through a .csv file inside the "menuconfig" tool.
The OpenCV modules takes a big portion of flash memory.


## Installation and Setup

Note: It is assumed that the reader already installed the ESP-IDF framework for working with ESP MCU targets, and know the basic command either with the CLI or for example the ESP-IDF VScode extension.
In order to use/change the application clone it inside a folder, then do the following steps:
- Select Flash Method, Port and target
- clean the micro-ros component through the "idf.py clean-microros" command
- run the "idf.py reconfigure" command, so all the XRCE-DDS changes written in the app-colcon.meta file will be updated correctly
- change the Wi-Fi settings using your SSID and password via the sdkconfig file or the "idf.py menuconfig" tool
- run "idf.py reconfigure" and then finally build the app with "idf.py build" command
- connect the board to the selected Port (e.g., /dev/tty/USB0) and Flash the application


More detailed instructions coming soon.. DB
