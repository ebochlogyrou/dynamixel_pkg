## How to Operate

**Important:** Before you turn on the dynamixels, make sure they are aligned with the markings on the nozzle!!!

1. Open a terminal (do not start roscore).
2. Launch the dynamixel_pkg:

    ```bash
    roslaunch dynamixel_pkg dynamic_launch.launch
    ```

3. Open a new terminal.
4. Publish a message to the `/normalvector_endeffector` topic. Replace the values in the `data` field with your input vector:

    ```bash
    rostopic pub -1 /normalvector_endeffector std_msgs/Float32MultiArray "layout:
      dim:
      - label: ''
        size: 0
        stride: 0
      data_offset: 0
      data: [1.0, 0, 1.0]" #replace with your input vector
    ```

**Note:** The inverse_kinematics node is not debugged yet.
