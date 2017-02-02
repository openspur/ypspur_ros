# ypspur_ros node

## Subscribed topics

### ~/cmd_vel [geometry_msgs/Twist]

Vehicle velocity control command input.

### ~/cmd_joint [trajectory_msgs/JointTrajectory]

Joint trajectory control command input.

### ~/control_mode [ypspur_ros/ControlMode]

Motor control mode input.
(velocity control, torque control, passive (short circuit) brake, and open circuit mode)

### ~/joint_position [ypspur_ros/JointPositionControl]

Joint position control command without time constraint. (Time optimal control.)

### ~/dio?_output [ypspur_ros/DigitalOutput]

Digital IO output command.
This topic name is configurable.

## Publihed topics

### ~/odom [nav_msgs/Odometry]

Odometry pose.

### ~/joint [sensor_msgs/JointState]

Joint status array.

### ~/wrench [geometry_msgs/WrenchStamped]

Estimated vehicle force and torque.

### ~/digital_input [ypspur_ros/DigitalInput]

Digital IO status.

### ~/ad/name [std_msgs/Float32]

Analog input status.
This topic name is configurable.

## Parameters

### ~/port [string: "/dev/ttyACM0"]

### ~/ypspur_bin [string: "/usr/local/bin/ypspur-coordinator"]

### ~/param_file [string: ""]

Path to your vehicle parameter file for yp-spur.

### ~/ad?_enable [bool: false]

### ~/ad?_name [string: "ad" + i]

Published topic name

### ~/ad?_gain [float: 1.0]

### ~/ad?_offset [float: 0.0]

### ~/dio?_enable [bool: false]

### ~/dio?_name [string: "dio" + i]

Subscribed topic name and published IO pin name.

### ~/dio?_output [bool: true]

### ~/dio?_input [bool: false]

### ~/dio?_default [string: "HIGH_IMPEDANCE"]

Default output state.

### ~/odom_id [string: "odom"]

Odometry frame id.

### ~/base_link_id [string: "base_link"]

base_link frame id.

### ~/hz [float: 200.0]

Frequency to publish odometry data.

### ~/odometry_mode [string: "diff"]

### ~/max_joint_id [int: 32]

### ~/joint?_enable [bool: false]

### ~/joint?_name [string: "joint" + i]

### ~/vel [float: max velocity specified in the parameter file]

### ~/acc [float: max acceleration specified in the parameter file]

### ~/angvel [float: max angular velocity specified in the parameter file]

### ~/angacc [float: max angular acceleration specified in the parameter file]



