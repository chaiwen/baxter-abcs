# COMSE 6731 HUMANOID ROBOTS, SPRING 2016
# Columbia University

## Installation
This project requires ROS to run, which should already be installed on several of the clic machines.

In addition to ROS, you will also need to install Portaudio, Pyaudio, and SpeechRecognition. Here is how to do it to your user directory on a clic machine:

1. Download portaudio from http://www.portaudio.com/download.html and save to folder portaudio in your home directory
2. ./configure --prefix=/home/UNI/portaudio_path
make
make install
3. Open ~/.pydistutils.cfg and save the following in that file:
[build_ext]
include_dirs=/home/UNI/portaudio_path/include/
library_dirs=/home/UNI/portaudio_path/lib/
4. pip install --user --allow-external pyaudio --allow-unverified pyaudio pyaudio
5. Pip install --user SpeechRecognition

## Getting Started

First, clone this repo.

For every new terminal you open, you will need to run:
```bash
$ source load_settings.sh
```

Then run catkin_make.

To generate the rest of the block models from block A, go into the models folder [baxter-abcs/src/system_launch/world/models] and run:

```bash
$ python generate_block_models.py
```

You should only need to do this once.


## Running the project

In separate terminals...

```bash
$ source load_settings.sh
$ roslaunch system_launch everything.launch
```

```bash
$ source load_settings.sh
$ rosrun baxter_tools enable_robot.py -e
$ rosrun control control.py
```

```bash
$ source load_settings.sh
$ rosrun vision kinect_processor
```

```bash
$ source load_settings.sh
$ rosrun control pick_up_blocks.py
```

```bash
$ source load_settings.sh
$ rosrun speech_command speech_command.py
```

## Other useful commands: 
```bash
$ killall gzserver         when system_launch dies and refuses to restart
$ ps aux | grep 'ros'      if the error 'port in use' comes up, run this to make sure no leftover ros processes, if there are any 'kill_all_ros.sh' should solve the problem
$ rosrun hello_baxter hello_baxter.py    run a script within a custom package 
```

