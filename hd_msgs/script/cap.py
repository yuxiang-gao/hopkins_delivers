#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
import datetime
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/image_view_1525979691267834454/output",Image,self.callback)
    self.count = 0
    self.dir = "./image"

  def callback(self,data):
    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
        '''cv_image = np.array(cv_image, dtype = np.dtype('f8'))
        for r in cv_image:
            for p in r:
                if np.isnan(p):
                    p = 0
        cv_image = cv2.normalize(cv_image, cv_image, 0, 1, cv2.NORM_MINMAX)
        cv_image = np.array(cv_image * 255, dtype=np.dtype(np.int32))
        '''
    except CvBridgeError as e:
        print(e)
    
    self.count = self.count + 1
    cv2.imwrite(
        self.dir + '/' + str(self.count) + '.jpg', cv_image)
    
def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

