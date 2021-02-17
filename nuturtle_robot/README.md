# nuturtle_robot package

This package run nodes on the turtlebot from your computer. It stores the code that interacts with the turtlebot hardware.  

# launch files

### basic_remote.launch  

The launch file first takes an argument <b>robot</b> to specify the hostname of the turtlebot. The turtlebot machine information is specified using the <b>machine</b> tag. By using the <b>machine</b> argument in <b>node</b> tag, the node can launch on turtlebot remotely.  

To launch node on my turtlebot Boba (make sure you can ping the hostname):
```
roslaunch nuturtle_robot basic_remote.launch robot:=<hostname>
```

### odom_teleop.launch

The launch file provides launches nodes remotely on the turtlebot3 and on current remote PC. It takes two arguments:  
<b>robot</b> - hostname of the turtlebot
<b>circle</b> - decides the velocity input

To launch node with ```turtlebot3_teleop_key``` control mode (make sure you can ping the hostname):  
```
roslaunch nuturtle_robot odom_teleop.launch robot:=boba.local
```
 
To launch node with ```follow_circle``` control mode (make sure you can ping the hostname):  
```
roslaunch nuturtle_robot odom_teleop.launch robot:=boba.local circle:=true
```




