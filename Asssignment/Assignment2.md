# Assignment2

## Default
```
ros2 run turtlesim turtlesim_node

ros2 run turtlesim turtle_teleop_key
```

## Problem1
![Assignment2-1](https://github.com/user-attachments/assets/a55b5b0a-3817-478f-ab17-32f1edccd8a3) <br/>
### Run bash code
```
bash assignment1.sh
```
### assignment1.sh
```
# First full loop (left larger loop, 7 units/sec)
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 6.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 6.28}}"  # Full circle (2π radians)
sleep 12  # Slightly increased to ensure the turtle completes the circle


# Second full loop (right smaller loop, 7 units/sec)
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 6.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -6.28}}"  # Full circle (2π radians, opposite direction)
sleep 12  # Slightly increased to ensure the turtle completes the circle


# Third full loop (left larger loop, 4 units/sec)
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 4.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 6.28}}"  # Full circle (2π radians)
sleep 7  # Increased sleep time for slower speed

# Fourth full loop (right smaller loop, 4 units/sec)
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 4.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -6.28}}"  # Full circle (2π radians, opposite direction)
sleep 5  # Increased sleep time for slower speed

```



## Problem2
![Assignment2-2](https://github.com/user-attachments/assets/68ff4690-0766-4bc4-a8a9-700824b24696) <br/>
```
bash assignment2.sh
```
### assignment2.sh
```
# Turn the turtle left 90 degrees to face north (counterclockwise)
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.57}}"  # Turn left 90 degrees (π/2 radians)
sleep 2  # Wait time for the 90-degree turn

# Stop before starting the second loop
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2  # Ensure complete stop (increased to 2 seconds)

# First full loop (right loop, 7 units/sec)
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 9.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -6.28}}"  # Full circle (2π radians)
sleep 11  # Wait time to complete the fifth loop

# Reset the turtle's orientation to face north (stop turning)
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2  # Ensure the turtle stops and faces north

# Second full loop (left loop, 7 units/sec)
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 9.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 6.28}}"  # Full circle (2π radians)
sleep 11  # Wait time to complete the sixth loop

# Final stop after all loops
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2  # Ensure the turtle stops

# Third full loop (right loop, 7 units/sec)
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 7.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -6.28}}"  # Full circle (2π radians)
sleep 11  # Wait time to complete the first loop

# Reset the turtle's orientation to face north (stop turning)
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2  # Ensure the turtle stops and faces north

# Fourth full loop (left loop, 7 units/sec)
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 7.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 6.28}}"  # Full circle (2π radians)
sleep 11  # Wait time to complete the second loop

# Reset the turtle's orientation to face north (stop turning)
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2  # Ensure the turtle stops and faces north

# Fifth full loop (right loop, 4 units/sec)
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 4.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -6.28}}"  # Full circle (2π radians)
sleep 20  # Longer wait time for slower speed

# Reset the turtle's orientation to face north (stop turning)
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2  # Ensure the turtle stops and faces north

# Sixtj full loop (left loop, 4 units/sec)
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 4.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 6.28}}"  # Full circle (2π radians)
sleep 20  # Longer wait time for slower speed

# Reset the turtle's orientation to face north (stop turning)
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2  # Ensure the turtle stops and faces north

```

## Problem3
![Assignment2-3](https://github.com/user-attachments/assets/c6d8d0f2-f984-48dd-b420-0149403237c7) <br/>
```
bash assignment3.sh
```
### assignment3.sh
```
# Final stop after all loops
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2  # Ensure the turtle stops

ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2

# Turn the turtle left 90 degrees to face north (counterclockwise)
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.57}}"  # Turn left 90 degrees (π/2 radians)
sleep 8  # Wait time for the 90-degree turn

ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 6.28}}"

# First full loop (right loop, 7 units/sec)
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 9.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -6.28}}"  # Full circle (2π radians)
sleep 11  # Wait time to complete the fifth loop

# Reset the turtle's orientation to face north (stop turning)
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 6.28}}"
sleep 2  # Ensure the turtle stops and faces north

# Second full loop (left loop, 7 units/sec)
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 9.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 6.28}}"  # Full circle (2π radians)
sleep 11  # Wait time to complete the sixth loop

# Final stop after all loops
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 6.28}}"
sleep 2  # Ensure the turtle stops
```
