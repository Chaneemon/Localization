# Guide_Konkuk

# How To Run Algorithm
1. sh kkanbu.sh
2. open another terminal
3. go into arduino/libraries
"rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600"

# File Tree

	└── KKANBU
		├── kkanbu_communication
		├── kkanbu_msgs
		├── kkanbu_planning
		|		├── global_planninng
		|		├── local_planning
		|		└── kkanbu_control
		├── kkanbu_sensors
		├── kkanbu_tf
		└── kkanbu_visualization


# Global Planning 

Input 1 : Ego position of robot from GNSS               [Start Point]

Input 2 : Goal Index from ondemand service (Customer)   [Goal Point]

Output : Global Path                                    [nav_msgs::Path]

1. Get pre-saved waypoint and parsing for Global Planning 
2. Extract "Landmark point" from parsed waypoint 
3. Use "A* Algorithm" for searching path from start to goal
4. Interpolate Path from A*
5. Publish Path to Local Planning

# Local Planning 

Input 1 : Ego position of robot from GNSS               [Start Point]

Input 2 : Global Path from Global Planning              [Standard Path]

Input 3 : 2D LiDAR data from Perception                 [Point Cloud Data]

Output : Best Local Path robot will be followed 

1. Get Closest WP and Set Lookahead Point for make local path
2. Make Several path adding some lateral offset
3. Collision Checking for paths
4. Costing paths with not collision
5. Selecting Best Path and publish

# Control 

Input 1 : Vehicle State from GNSS - position, velocity,yaw, yawrate?

Input 2 : Best Path from Local Planning

Output : Lateral, Longitudinal Control Value to Arduino

1. Get Closest WP and Set Lookahead Point for Control
2. PID Control for Longitudinal Control --> Reinforcement Learning?
3. Pure Pursuit Algorithm for Lateral Control 
4. Publish to Arduino (Using ROS Serial)
