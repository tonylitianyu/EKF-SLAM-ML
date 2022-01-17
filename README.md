# Landmark-based EKF-SLAM with Machine Learning from Scratch
* Tianyu Li
* Started Winter 2021

* For project description, please visit [project link](http://imtianyuli.com/projects/2021/10/23/ekfslam.html)

# Package List
This repository consists of several ROS packages
- nuturtle_description - Adapts the model of turtlebot3 for our needs and display it in rviz
- rigid2d - A library for performing 2D rigid body transformations
- trect - Makes a turtle in the turtlesim to move in a rectangular trajectory
- nuturtle_robot - Provides interface and interaction with turtlebot hardware
- nurtlesim - Creates kinematic simulation for a diff drive robot
- nuslam - Runs the Extended Kalman Filter SLAM

# Dependencies
```
nuturtlebot (for urdf)
https://github.com/ME495-Navigation/nuturtlebot.git

catch_ros (for testing)
https://github.com/AIS-Bonn/catch_ros.git
```

# Launch

To run the SLAM in rviz with known data association
```
roslaunch nuslam slam.launch
```

To run the SLAM in rviz with unknown data association
```
roslaunch nuslam unknown_data_assoc.launch
```

(More)
