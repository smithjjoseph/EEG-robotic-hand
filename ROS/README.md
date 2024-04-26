# ROS

### Instruction
1. Finished Following [installation instructions](http://wiki.ros.org/noetic/Installation/Ubuntu)
2. Information on general [Joint Trajectory Action](http://wiki.ros.org/pr2_controllers/Tutorials/Moving%20the%20arm%20using%20the%20Joint%20Trajectory%20Action)
3. Information on qbhand [Waypoint Control](https://wiki.ros.org/qb_hand_control/Tutorials/Waypoint%20Control)

### ROS Setup
Within .bashrc a script is called and some env vars are setup for use with ROS
- `setup.bash`: Initialises catkin (low-level build system/infrastructure for ROS)
- `ROS_MASTER_URI`: Required so nodes know where the master is
    - Should be set to XML-RPC URI of master
    - `=http://172.16.0.1:11311`
- `ROS_HOSTNAME`/`ROS_IP`: Set the declared network address of a node
    - Mutually exclusive - hostname takes precedence over IP 
    - Use `ROS_IP` for IPs and `ROS_HOSTNAME` for host names
    - `=172.16.0.5`

### Command Line Tools
- `rostopic`: Interacts with ROS topics (publishers, subscribers, publishing rates and ROS messages)
    - `rostopic list` returns:
        - `/qbhand2m1/control/joint_states`
        - `/qbhand2m1/control/qbhand2m1_motor_positions_trajectory_controller/command`
        - `/qbhand2m1/control/qbhand2m1_motor_positions_trajectory_controller/follow_joint_trajectory/cancel`
        - `/qbhand2m1/control/qbhand2m1_motor_positions_trajectory_controller/follow_joint_trajectory/feedback`
        - `/qbhand2m1/control/qbhand2m1_motor_positions_trajectory_controller/follow_joint_trajectory/goal`
        - `/qbhand2m1/control/qbhand2m1_motor_positions_trajectory_controller/follow_joint_trajectory/result`
        - `/qbhand2m1/control/qbhand2m1_motor_positions_trajectory_controller/follow_joint_trajectory/status`
        - `/qbhand2m1/control/qbhand2m1_motor_positions_trajectory_controller/state`
        - `/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/command`
        - `/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/follow_joint_trajectory/cancel`
        - `/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/follow_joint_trajectory/feedback`
        - `/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/follow_joint_trajectory/goal`
        - `/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/follow_joint_trajectory/result`
        - `/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/follow_joint_trajectory/status`
        - `/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/state`
        - `/qbhand2m1/control/startup_sync`
        - `/qbhand2m1/frequency`
        - `/qbhand2m1/joint_states`
        - `/qbhand2m1/qbhand2m1/state`
- `rosparam`: Interacts with ROS parameters

### Nomenclature/About
High-level Script → Action Interface → Controller → Robot Joints

- **High-level Script:** Uses action interface to provide desired joint trajectories
- **Action Interface:** ROS action which takes a trajectory command expressed as a series of joint angles and sends corresponding low-level commands to the controller
- **Controller:** Sends commands directly to joints

Joint Trajectory Message: Header + Joint Names + Joint Trajectory Points

- [**Joint Trajectory Message**](http://docs.ros.org/en/api/trajectory_msgs/html/msg/JointTrajectory.html), `JointTrajectory`: Contains a header, set of joint names, and a set of waypoints containing: pos, vel, acc at each joint
- [**Header**](http://docs.ros.org/en/api/std_msgs/html/msg/Header.html): Contains sequence ID, timestamp, and ID for association with a data frame - It appears sequence ID and frame ID are handled by client library 
- **Joint Names**: List of strings containing joint names
- [**Joint Trajectory Information**](http://docs.ros.org/en/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html), `JointTrajectoryPoint`: Contains values of position, velocity and acceleration for each joint included in the joint names list (uses same order) as well as a time from start

https://github.com/NMMI/ROS-SoftHand?tab=readme-ov-file#2-waypoint-control