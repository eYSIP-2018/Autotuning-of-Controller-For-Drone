# Autotuning-of-Controller-For-Drone-


**Follow the below steps to use packages for manual tuning and auto-tuning based on Ziegler-Nichols method**

Step 1: Clone the folders magnetometer_check, pluto_drone, task_3. The details of these packages can be found in their respective folders.

Step 2: In a terminal run the command **_roslaunch magnetometer_check mag.launch_**. Follow the instructions given in the readme of this package to check if the magnetometer of drone is working properly.

Step 3: Once you are sure of the magnetometers correct functioning, on a new terminal run the command **_roslaunch                             task_3.launch_**

Step 4: On a new terminal run the command **_roslaunch plutoserver drone_comb.launch_**

Step 4: Check if the drone is working correctly by pressing 'a'(without quotes) to arm it and 'd' to disarm it.

Step 5 : For manually tuned waypoint navigation/position holding run the command **_rosrun plutoserver PID-controller.py_**
for auto-tuned waypoint navigation **_rosrun plutoserver auto-tuning-doc.py_** on a new terminal.

**Follow the below steps to use packages for Iteration Based Auto-Tuning **

Step 1: In a terminal run the command **_roslaunch magnetometer_check mag.launch_**. Follow the instructions given in the readme of this package to check if the magnetometer of drone is working properly.

Step 2: Once you are sure of the magnetometers correct functioning, on a new terminal run the command **_roslaunch                            task_3.launch_**

Step 3: On a new terminal use command **_rosrun plutoserver Auto_Tuning.py_** to start the auto tuning of pid controller for drone.
