Teleoperation Control of ROS-based Industrial Robot with EMG signals.

This System is used in ROS in Ubuntu environments (PC). Robot Arm with 6 DOF is controlled remotely using a EMG sensor. 
Robot arms and PC are connected, and the sensor is wirelessly connected to PC using a Bluetooth adapter. 
Using ROS-communication, arm movement data recognized by the sensor are sent to the robot arm. 
Gesture classifiers were created from data collected directly by specific personnel using the Myo armband.

The robot arm moves in the same direction as the human movement.
And a gripper is operated by a human hand gesture.


****System Structure****
![시스템 구상도2](https://user-images.githubusercontent.com/60131899/95708960-976d0a00-0c98-11eb-9fc4-d20ca56bdbd3.png)

**RQT graph Summary**
![rqt요약](https://user-images.githubusercontent.com/60131899/95709508-bd46de80-0c99-11eb-85ed-f48bd84180ad.PNG)


**Development Environment and Equipment**


Robot arm : UR3 (Universal Robots)

Gripper : 2F - 140 Gripper (Robotiq)

OS : Linux Ubuntu 18.04

ROS : ROS Melodic Morenia

Sensor : Myo armband (Thalmic Labs)




**Working System**(in Ubuntu)


$ roscore

$ rosrun ros_myo myo-rawNode.py

$ rosrun ur_robot_driver listener3.py

(Virtual environment based on Python 3.6 version)
$ python pred-echo-server.py

$ roslaunch ur_robot_driver ur3_bringup.launch robot_ip:= (input_your_ip)

$ rosrun ur_robot_driver uremg.py

$ rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0

$ rosrun robotiq_2f_gripper_control Robotiq2FGripperStatusListener.py

(Virtual environment based on Python 2.7 version)
$ rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController2.py


Make your gesture !


