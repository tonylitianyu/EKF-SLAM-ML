# Turtle Rect
* A package that causes the turtlesim turtle to follow a rectangle.
# Example Usage
```
roslaunch trect trect.launch

rosservice call /start "init_x: 2.0                             
init_y: 3.0
width: 4.0
height: 5.0" 
```
![Demonstration](<trect.gif>)
