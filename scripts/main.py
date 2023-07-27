#!/usr/bin/env python3

import os
import time
from baxter_head import BaxterHead

package_directory = os.path.dirname(os.path.realpath(__file__)) + "/../"

if __name__ == "__main__":
    baxter_head = BaxterHead()

    delta_time = 0
    while not rospy.is_shutdown():
        time_last = time.time()
        baxter_head.update(delta_time)
        rospy.sleep(0.01)
        delta_time = time.time() - time_last

    baxter_head.close()