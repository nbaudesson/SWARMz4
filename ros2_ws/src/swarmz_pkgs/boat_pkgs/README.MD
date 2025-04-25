# BOAT PACKAGES

## Description
This is a folder which contains all the files needed to control the boat in the Gazebo simulation of ship and drone battle challenge.

## Features
- **Gazebo maritime:** source code needed to load plugins used by gazebo to simulate the boat's physics and control its behaviour
- **Cannon interfaces:** custom interfaces used for cannon action server
- **Boat driver:** python nodes for thruster and cannon control

## Cannon action server

This action server manage the control of the ship's cannon, it receives a goal request of type `cannon_interfaces/action/Cannon` which has the following format:

```bash
#Goal
float32 pitch
float32 yaw
string target_ship
```

- **Pitch:** Float with desired pitch
- **Yaw:** Float with desired yaw
- **Target_ship:** String with ship's namespace

While the request is being processed the server gives you back the feedback containing the current yaw and pitch of the ship

## Testing cannon server

The action request is defined with 3 parameters in the Cannon.action file :  
   - `pitch`: must be between [-1.57, 1.57]
   - `yaw`: must be between [-3.14, 3.14]
   - `target_ship` : name of the ship to control (example : flag_ship_1, flag_ship_2...)

Start the action server in a terminal: 

```bash
# Activate cannon controller server
ros2 run boat_driver cannon_server
```

You can then send a goal request to the controller with the following command:

```bash
# Send a goal request to cannon controller server, you specify the target ship, yaw, and pitch desired
ros2 action send_goal /cannon cannon_interfaces/action/Cannon "{'pitch': 1.0, 'yaw': -1.0, 'target_ship': 'flag_ship_1'}" --feedback
```

When you send a goal using the action client in the terminal, you will receive feedback showing the current pitch and yaw orientations of the cannon, with an average speed of 0.3 rad/s.


## Boat client test

This package also contains a python client node called **boat_test** to test all the features of the ship, this node executes a sequence of tasks which are designed to test the cannon control, the missile firing, and the thrusters of the ship with a basic go to goal algorithm. You can take this code as an example to design your own boat controller node, to test it you can use the following command: 

```bash
# Launch boat test node
ros2 launch boat_driver ship_test_launch.py team_id:="2" tolerance_angle:="1.0" tolerance_distance:="5.0"

# or you can just run the node directly with the values by default
ros2 run boat_driver boat_test
```

After you run the command you should see the ship executing a sequence of 4 different cannon positions and shooting each time the desired position is reached, then the ship will move to the center of the respective spawn area. If you check the code you will find the topics that are being used to publish the desired commands for your thrusters `/model/flag_ship_{team_id}/joint/left_engine_propeller_joint/cmd_thrust` and `/model/flag_ship_{team_id}/joint/right_engine_propeller_joint/cmd_thrust` replacing {team_id} for your team's number (1 or 2) like this:

```bash
#Send velocity command to left thruster of team1 boat
ros2 topic pub /model/flag_ship_1/joint/left_engine_propeller_joint/cmd_thrust std_msgs/msg/Float64 "data: 2.0" --once
```

**NOTE:** You should take into account the physics of the simulation, in other words, your boat is floating in a moving ocean with waves which means the boat will be moving even if the thrusters are not running, this movement is not that significant but it should not be ignored over time, so you should regularly check your pose and adjust the velocity to maintain your position and orientation. You can get the current pose of you ship in the topic `/flag_ship_{team_id}/pose` (you must have a game master running to be able to get the pose)

```bash
#Get position and orientation of boat 1
ros2 topic echo /flag_ship_1/pose
```