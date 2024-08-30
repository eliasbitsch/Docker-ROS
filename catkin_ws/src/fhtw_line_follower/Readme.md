# FHTW line follower ROS package


## Start the project

To start the project, follow these steps:

1. Navigate to the scripts folder.
```bash
cd ~/catkin_ws/src/fhtw/fhtw_line_follower/scripts
```


2. Start the start.py script.
```bash
python3 start.py
```

## Switch steering modes

To switch steering modes, set the parameter `/motion_node/steering_mode` accordingly in another terminal. This is only possible if the robot has reached the end of the maze.

### Bang bang

```bash
rosparam set /motion_node/steering_mode bang_bang
```

### Proportional

```bash
rosparam set /motion_node/steering_mode proportional
```

## Quit

To quit the script, press `ctrl+c` in the terminal where the script is running.