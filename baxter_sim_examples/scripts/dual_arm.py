import sys
import rospy
from std_msgs.msg import (
    UInt16,
)
import baxter_interface
from baxter_interface import CHECK_VERSION

class PickandPlace(object):
  
  def __init__(self, limb, pose):
   
