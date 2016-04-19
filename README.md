# COMSE 6731 HUMANOID ROBOTS, SPRING 2016
# Columbia University


## Installation
Everything you need has already been installed on the CLIC lab machines.  If you are trying to install it on your own machine, please follow the instructions here:
http://sdk.rethinkrobotics.com/wiki/Simulator_Installation

## Getting Started

The following lines will not work unless ROS is properly installed.  You can find instructions for how to do this from the ros_tutorial pdf on the class website.

```bash
$ cd ~
$ git clone git@github.com:HumanoidRobotics/baxter_example_code.git
$ cd baxter_example_code
$ source /opt/ros/indigo/setup.bash
$ catkin_make
$ source load_settings.sh

Enable the robot (in another terminal window):
$ rosrun baxter_tools enable_robot.py -e
```

## Running the Demo code
First, bring up Gazebo, and Baxter
(just run this, it launches Gazebo for us)
```bash
$  ./baxter.sh sim
$ roslaunch system_launch everything.launch
```

Then run the individual demos with any of the following:
```bash
$ source devel/setup.bash
$ ./baxter.sh sim
Then:
$ rosrun baxter_examples joint_position_keyboard.py
$ rosrun baxter_examples joint_velocity_wobbler.py
```

Note: For every new terminal you open up, you will have to run the following:
```bash
$ source devel/setup.bash
$ ./baxter.sh sim
```
Note: in the simulator, all 3 baxter cameras are on at once! Don't worry about turning them on and off. But for real camera, only use images from 1 hand.
Use "rosrun rviz rviz" because it's useful! When adding a camera, create a visualization by topic, under the image folder pick "Image" to stream what the camera sees. 

Useful commands: 
```bash
$ killall gzserver         when system_launch dies and refuses to restart
$ ps aux | grep 'ros'      if the error 'port in use' comes up, run this to make sure no leftover ros processes, if there are any 'kill_all_ros.sh' should solve the problem
$ rosrun hello_baxter hello_baxter.py    run a script within a custom package 
```

## FYI:
For every new terminal, if you want our models (aka the blocks) to show up, you have to do:
```bash
$ export GAZEBO_MODEL_PATH=/home/your_uni/baxter-abcs/src/system_launch/world/models
$ export GAZEBO_RESOURCE_PATH=/home/your_uni/baxter-abcs/src/system_launch/world/models
```
Or add those to your devel/setup.bash since we source that every time anyway. Be sure to change the path to your uni's path.

Ok we have a new thing now instead of sourcing devel/setup.bash just source load_settings.sh instead. it'll take care of the export stuff too!

## Generating Block Models for Gazebo:
To generate the rest of the block models from block A, go into the models folder [baxter-abcs/src/system_launch/world/models] and run "python generate_block_models.py" You should only need to do this once

