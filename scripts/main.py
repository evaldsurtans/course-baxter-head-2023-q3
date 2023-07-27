#!/usr/bin/env python3
import math
import os
import time

import rospy # pip3 install --extra-index-url https://rospypi.github.io/simple/ rospy
import cv2
import argparse

import pyrosenv.cv_bridge # pip3 install pyrosenv
from pyrosenv.sensor_msgs.msg import Image

from baxter_interface import CHECK_VERSION
from baxter_interface import Head
from baxter_interface import CameraController

package_directory = os.path.dirname(os.path.realpath(__file__)) + "/../"
# TODO