import math
import os
import time

import rospy # pip3 install --extra-index-url https://rospypi.github.io/simple/ rospy
import argparse

from sensor_msgs.msg import Image
from std_msgs.msg import UInt16

import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'

if ros_path in sys.path:
    sys.path.remove(ros_path)

import cv2

sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_interface import Head
from baxter_inteace import CameraController

package_directory = os.path.dirname(os.path.realpath(__file__)) + "/../"

class BaxterHead:
    def __init__(self, eyes_anim='blink'):
        super(BaxterHead, self).__init__()

        rospy.init_node("baxter_head")
        rospy.loginfo("Initializing Baxter Head")

        self.bax_main = baxter_interface.RobotEnable(CHECK_VERSION)
        if not self.bax_main.state().enabled:
            self.bax_main.enable()

        self.bax_head = Head()
        self.bax_head.set_pan(angle=0)

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
        path_images = package_directory + "resources/eyes/" + eyes_anim + "/"
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
                img = cv2.resize(img, (1024, 600))

                msg = Image()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "camera_frame"

                msg.height = img.shape[0]
                msg.width = img.shape[1]
                msg.encoding = "bgr8"
                msg.is_bigendian = False
                msg.step = 3 * msg.width
                msg.data = img.tostring()

                self.pub_display.publish(msg)

                self.state_eyes_idx = (self.state_eyes_idx + 1) % len(self.state_eyes_images)
                self.state_eyes_delta_time = 0

    def update(self, delta_time):
        self.update_eyes_animation(delta_time)