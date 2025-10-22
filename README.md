# tomato_sorting_system_ur10e
## Automated Tomato Ripeness Sorting System with UR10e and OAK-D camera at VGU Robotics Lab
### ROS 1 Noetic and YOLOv8 for Object Detection
**"All great things have small beginnings." — Michael Fassbender**\
\
This project is my Bachelor Thesis conducted at VGU Robotics Lab for the B.Eng EEIT program of Frankfurt UAS in cooperation with Vietnamese-German University. Although the idea and codings are simple, I hope it would be a great start!\
\
In this project, I also implemented simple VR Control, which code can be found in the <ins>main_vr.py</ins>. UR10e will remember and follow the paths that are set by the VIVE VR Controller. Codes for the VR can be found in [vive_ur10e](https://github.com/trungtran22/vive_ur10e).\
\
The camera for this project is OAK-D Depth Camera. For more information about ROS and OAK-D, it can be found here [depthai_ros](https://github.com/luxonis/depthai-ros). The blob file for the tomato can be found in the tomato_data folder. The code for the camera is available in the script folder.\
![](https://github.com/trungtran22/tomato_sorting_system_ur10e/blob/main/Pics/OAK.png)\
\
>A few pictures of the project
**Robot System Setup**\
![](https://github.com/trungtran22/tomato_sorting_system_ur10e/blob/main/Pics/Robot_Sys.png)
\
**Conveyor Belt Setup and Wiring**\
![](https://github.com/trungtran22/tomato_sorting_system_ur10e/blob/main/Pics/Conveyor.png)
![Wiring](https://github.com/trungtran22/tomato_sorting_system_ur10e/blob/main/Pics/Conveyor_Electrical.png)
\
**Tomato Class**
![](https://github.com/trungtran22/tomato_sorting_system_ur10e/blob/main/Pics/Tomato_class.png)
