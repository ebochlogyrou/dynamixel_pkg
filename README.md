
This is a provisional branch where I merged the inverse kinematics with the rest. 
How to operate: 
Important: before you turn on the dynamixels on, make sure you have them alligned with the markings on the nozzle!!!
1. open terminal(don't open roscore)
2. roslaunch dynamixel_pkg dynamic_launch.launch
3. new terminal
4. rostopic pub -1 /normalvector_endeffector std_msgs/Float32MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [1.0, 0, 1.0]" #replace with your input vector
-----------------------------------------------------------------------------------------
kinematics don't work yet
