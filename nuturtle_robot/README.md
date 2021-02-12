# nuturtle_robot package

This package run nodes on the turtlebot from your computer. It stores the code that interacts with the turtlebot hardware.  

# launch file

- The launch file first takes an argument <b>robot</b> to specify the hostname of the turtlebot.
- The launch file then specifies the turtlebot machine information using the <b>machine</b> tag.
- The launch file also load node using the turtlebot remotely using the <b>machine</b> argument in <b>node</b> tag.