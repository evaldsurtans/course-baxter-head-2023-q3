import math
import os
import time

import numpy as np
import rospy # pip3 install --extra-index-url https://rospypi.github.io/simple/ rospy
import argparse

from sensor_msgs.msg import Image
from std_msgs.msg import UInt16

import matplotlib.pyplot as plt

import pdb

import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'

if ros_path in sys.path:
    sys.path.remove(ros_path)

import cv2

sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

from baxter_core_msgs.msg import NavigatorState
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_interface import Head
from baxter_interface import CameraController

package_directory = os.path.dirname(os.path.realpath(__file__)) + "/../"

class BaxterHead:
    def __init__(self, eyes_anim='blink'):
        super(BaxterHead, self).__init__()

        rospy.init_node("baxter_head")
        rospy.loginfo("Initializing Baxter Head")

        self.eyes_anim = eyes_anim

        self.bax_main = baxter_interface.RobotEnable(CHECK_VERSION)
        if not self.bax_main.state().enabled:
            rospy.loginfo("Enabling Baxter")
            self.bax_main.enable()
        else:
            rospy.loginfo("Baxter already enabled")

        self.bax_head = Head()
        rospy.loginfo("pan: %f" % self.bax_head.pan())
        rospy.loginfo("panning: %s" % self.bax_head.panning())
        rospy.loginfo("nodding: %s" % self.bax_head.nodding())
        #self.bax_head.command_nod()
        rospy.loginfo("Panning head to 0")
        self.bax_head.set_pan(angle=0.0)

        rospy.loginfo("Initializing head camera")
        self.head_cam = CameraController('head_camera')
        # try:
        #     self.head_cam = CameraController('head_camera')
        #     self.head_cam.resolution = (1280, 800)
        #     self.head_cam.open()
        # except AttributeError:
        #     self.head_cam = None
        #     rospy.loginfo("Camera already initialized, closing left hand camera")
        #     left_cam = CameraController('left_hand_camera')
        #     left_cam.close()
        #
        # if not self.head_cam:
        #     rospy.loginfo("Initializing head camera")
        #     self.head_cam = CameraController('head_camera')
        #     self.head_cam.resolution = (1280, 800)
        #     self.head_cam.open()
        #     rospy.loginfo("Head camera initialized")

        self.sub_head_cam = rospy.Subscriber(
            '/cameras/head_camera/image', Image, self.on_head_cam, queue_size=1)

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

        self.sub_nav_arm_left = rospy.Subscriber(
            'robot/navigators/left_navigator/state',
            NavigatorState,
            self.on_nav_arm_left,
            queue_size=1
        )


        # disable sonar
        rospy.loginfo("Disabling sonar")
        self.pub_head_sonar.publish(0)

        self.state_eyes_images = []
        self.state_eyes_idx = 0
        self.state_eyes_delta_time = 0
        self.set_eyes_animation(eyes_anim)

        # Field of view
        self.CENTER_X = int(640 / 2)
        self.FOV = math.pi / 3
        self.FACE_RANGE = 0

        self.cv_face_cascade = cv2.CascadeClassifier(
            package_directory + 'resources/opencv/haarcascade_frontalface_default.xml')

        self.limb_left = baxter_interface.Limb('left')
        self.gripper_left = baxter_interface.Gripper('left',  CHECK_VERSION)
        self.stored_joint_angles_left = None

    def close(self):
        rospy.loginfo("Closing Baxter Head")
        #self.head_cam.close()
        #self.bax_main.disable()

    def on_nav_arm_left(self, msg):
        if any(msg.buttons):
            rospy.loginfo("on_nav_arm_right button pressed")
            self.bax_head.command_nod()
            if self.stored_joint_angles_left is None:
                rospy.loginfo("on_nav_arm_right storing joint angles")
                self.stored_joint_angles_left = self.limb_left.joint_angles()
            else:
                rospy.loginfo("on_nav_arm_right moving to stored joint angles")
                self.limb_left.move_to_joint_positions(self.stored_joint_angles_left)
                self.stored_joint_angles_left = None


    def on_head_cam(self, msg):

        self.sub_head_cam.unregister()

        #rospy.loginfo("Received head camera image")

        img = np.fromstring(msg.data, np.uint8)
        img = img.reshape(msg.height, msg.width, 4)

        rospy.loginfo("Image shape: " + str(img.shape))

        # cv2.imshow('head_camera', img)
        # cv2.waitKey(0)
        # return


        gray = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)

        # plt.close()
        # plt.imshow(gray, cmap='gray')
        # plt.show()
        #cv2.imshow('gray', gray)
        # return

        faces = self.cv_face_cascade.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=3, minSize=(10, 10), flags=2)

        # closest face
        dif_x = float('inf')
        is_face = False

        rospy.loginfo("Found %d faces" % len(faces))

        # Iterating through all faces
        for (x, y, w, h) in faces:
            # cut face
            #face = img[y:y + h, x:x + w]
            # cv2.imshow('face', face)
            # cv2.waitKey(1)

            # draw box on gray image
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0, 0), 2)

            cur_pan = self.bax_head.pan()


            temp_dif_x = (x + (w / 2)) - self.CENTER_X
            # If the face is closer than the last one found, then set it
            # as the face to go to
            if temp_dif_x < dif_x:
                is_face = True
                dif_x = temp_dif_x

        #cv2.imshow('faces', img)
        # plt.close()
        # plt.imshow(img)
        # plt.show()


        if is_face:
            self.set_eyes_animation('happy')
            if dif_x > self.FACE_RANGE:
                self.bax_head.set_pan(
                    angle=cur_pan + -1 * (dif_x * (self.FOV / 2)) /
                          self.CENTER_X)
            elif dif_x < (-1 * self.FACE_RANGE):
                self.bax_head.set_pan(
                    angle=cur_pan + -1 * (dif_x * (self.FOV / 2)) /
                          self.CENTER_X)
        else:
            self.set_eyes_animation('processing')
            self.bax_head.set_pan(angle=0.0)

        self.sub_head_cam = rospy.Subscriber(
            '/cameras/head_camera/image', Image, self.on_head_cam, queue_size=1)

    def set_eyes_animation(self, eyes_anim):
        if self.eyes_anim != eyes_anim:
            self.eyes_anim = eyes_anim
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
                # cv_bridge - do not use on python3

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