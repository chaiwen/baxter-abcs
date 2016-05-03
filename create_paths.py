#!/usr/bin/env python

import os

path = os.getcwd()

with open("./src/vision/src/kinect_processor.cpp", 'r') as kinect_file:
    kinect_file_text = kinect_file.read().replace("__PATH__", path)

with open("./src/vision/src/kinect_processor.cpp", 'w') as kinect_file:
    kinect_file.write(kinect_file_text)
