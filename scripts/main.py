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

from baxter_head import BaxterHead

package_directory = os.path.dirname(os.path.realpath(__file__)) + "/../"

if __name__ == "__main__":
    baxter_head = BaxterHead()

    time_last = time.time()
    while not rospy.is_shutdown():
        delta_time = time.time() - time_last
        baxter_head.update(delta_time)
        rospy.sleep(0.01)
        time_last = time.time()

    baxter_head.close()