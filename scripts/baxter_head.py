import math
import os
import time

import rospy # pip3 install --extra-index-url https://rospypi.github.io/simple/ rospy
import cv2
import argparse

import pyrosenv.cv_bridge # pip3 install pyrosenv
from pyrosenv.sensor_msgs.msg import Image
from pyrosenv.std_msgs.msg import UInt16

from baxter_interface import CHECK_VERSION
from baxter_interface import Head
from baxter_interface import CameraController

package_directory = os.path.dirname(os.path.realpath(__file__)) + "/../"

class BaxterHead:
    def __init__(self, eyes_anim='blink'):
        super(BaxterHead, self).__init__()

        rospy.init_node("baxter_head")
        rospy.loginfo("Initializing Baxter Head")

        self.bax_main = baxter_interface.RobotEnable(CHECK_VERSION)
        if not self.bax_main.state().enabled:
            self.bax_main.enable()

        self.pub_display = rospy.Publisher(
            "/robot/xdisplay",
            Image,
            latch=True,
            queue_size=1
        )

        self.pub_head_sonar = rospy.Publisher(
            "/robot/sonar/head_sonar/set_sonars_enabled",
            UInt16,
            latch=True,
            queue_size=1
        )

        # disable sonar
        rospy.loginfo("Disabling sonar")
        self.pub_head_sonar.publish(0)

        self.state_eyes_images = []
        self.state_eyes_idx = 0
        self.state_eyes_delta_time = 0
        self.set_eyes_animation(eyes_anim)

    def close(self):
        rospy.loginfo("Closing Baxter Head")
        self.bax_main.disable()

    def set_eyes_animation(self, eyes_anim):
        rospy.loginfo("Setting eyes animation to: " + eyes_anim)
        path_images = package_directory + "images/eyes/" + eyes_anim + "/"
        self.state_eyes_images = sorted(os.listdir(path_images))
        self.state_eyes_images = [path_images + x for x in self.state_eyes_images]
        self.state_eyes_idx = 0

        rospy.logdebug("Loaded " + str(len(self.state_eyes_images)) + " images")
        rospy.logdebug("First image: " + self.state_eyes_images[0])

    def update_eyes_animation(self, delta_time):
        if len(self.state_eyes_images) > 0:
            self.state_eyes_delta_time += delta_time
            if self.state_eyes_delta_time > 0.1:
                img = cv2.imread(self.state_eyes_images[self.state_eyes_idx])
                msg = pyrosenv.cv_bridge.cv2_to_imgmsg(img, encoding="bgr8")
                self.pub_display.publish(msg)

                self.state_eyes_idx = (self.state_eyes_idx + 1) % len(self.state_eyes_images)
                self.state_eyes_delta_time = 0

    def update(self, delta_time):
        self.update_eyes_animation(delta_time)