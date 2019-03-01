import sys
import rospy
import rospkg
import time
import struct
import sys
import copy
import argparse
import threading

from std_msgs.msg import (
    UInt16,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface
from baxter_interface import CHECK_VERSION

class PickandPlace(object):
	def __init__(self, hover_distance = 0.1, verbose = True):
		#initialise arms and grippers
		self._left_arm = baxter_interface.Limb("left")
		self._right_arm = baxter_interface.Limb("right")
		self._left_gripper = baxter_interface.Gripper("left")
		self._right_gripper = baxter_interface.Gripper("right")
		self._verbose = verbose # bool
		ns_l = "ExternalTools/left/PositionKinematicsNode/IKService"
		ns_r = "ExternalTools/right/PositionKinematicsNode/IKService"
		self._iksvc_l = rospy.ServiceProxy(ns_l, SolvePositionIK)
		rospy.wait_for_service(ns_l, 5.0)
		self._iksvc_r = rospy.ServiceProxy(ns_r, SolvePositionIK)
		rospy.wait_for_service(ns_r, 5.0)
		
		print("Getting robot state...")
		self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
		self._init_state = self._rs.state().enabled
		print("Enabling robot...")
		self._rs.enable()
		
	def set_neutral(self):
		#set both arms to a neutral pose
		
		print("Moving to neutral pose...")
		self._left_arm.move_to_neutral()
		self._right_arm.move_to_neutral()
		
	def clean_shutdown(self):
		print("#nExiting programme...")
		self.set_neutral()
		if not self._init_state:
			print("Disabling robot...")
			self._rs.disable()
		return True
	
	def move_to(self, arm_name, joint_positions):
		#left_joint_positions = ik_request_left.pose
		#right_joint_positions = ik_request_right.pose
		#rate = rospy.Rate(100)
		if arm_name == "left":
			self._left_arm.move_to_joint_positions(joint_positions)
		if arm_name == "right":
			self._right_arm.move_to_joint_positions(joint_positions)
		else:
			rospy.logerr("No Joint Angles, staying put.")
	
	def ik_request_left(self, pose):
		hdr = Header(stamp=rospy.Time.now(), frame_id='base')
		ikreq = SolvePositionIKRequest()
		ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
		try:
			resp = self._iksvc_l(ikreq)
		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))
			return False
	# Check if result valid, and type of seed ultimately used to get solution
		# convert rospy's string representation of uint8[]'s to int's
		resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
		limb_joints = {}
		if (resp_seeds[0] != resp.RESULT_INVALID):
			seed_str = {
						ikreq.SEED_USER: 'User Provided Seed',
						ikreq.SEED_CURRENT: 'Current Joint Angles',
						ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
		   				}.get(resp_seeds[0], 'None')
			if self._verbose:
				print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
			 		(seed_str)))
				# Format solution into Limb API-compatible dictionary
			limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
			if self._verbose:
				print("IK Joint Solution:\n{0}".format(limb_joints))
				print("------------------")
		else:
			rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
			return False
		return limb_joints
	
	def ik_request_right(self, pose):
		hdr = Header(stamp=rospy.Time.now(), frame_id='base')
		ikreq = SolvePositionIKRequest()
		ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
		try:
			resp = self._iksvc_r(ikreq)
		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))
			return False
	# Check if result valid, and type of seed ultimately used to get solution
		# convert rospy's string representation of uint8[]'s to int's
		resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
		limb_joints = {}
		if (resp_seeds[0] != resp.RESULT_INVALID):
			seed_str = {
				ikreq.SEED_USER: 'User Provided Seed',
				ikreq.SEED_CURRENT: 'Current Joint Angles',
				ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
		   		}.get(resp_seeds[0], 'None')
		if self._verbose:
			print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
			 	(seed_str)))
			# Format solution into Limb API-compatible dictionary
		limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
		if self._verbose:
			print("IK Joint Solution:\n{0}".format(limb_joints))
			print("------------------")
		else:
			rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
			return False
		return limb_joints
		
 	def gripper_open_left(self):
		self._left_gripper.open()
		rospy.sleep(1.0)

	def gripper_open_right(self):
		self._right_gripper.open()
		rospy.sleep(1.0)

	def gripper_close_left(self):
		self._left_gripper.close()

	def gripper_close_right(self):
		self._right_gripper.close()

class myThread (threading.Thread):
	def __init__(self,threadID, name, loc):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.loc = loc
	def run(self):
		if self.name == "left":
			joints = pnp.ik_request_left(self.loc)
		if self.name == "right":
			joints = pnp.ik_request_right(self.loc)
		pnp.move_to(self.name, joints)

rospy.init_node("ik_pick_and_place_demo")

overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)
spwan_loc = Pose(
        position=Point(x=0.75, y=0, z=0.18),#y=0.227,z=0.169
        #position=Point(x=0.6725, y=0.22, z=0.8210),#-0.129),
        orientation= overhead_orientation)
pnp = PickandPlace()
pnp.set_neutral()

left_joints = pnp.ik_request_left(spwan_loc)
right_joints = pnp.ik_request_right(spwan_loc)

thread1 = myThread(1, "left", spwan_loc)
thread2 = myThread(2, "right", spwan_loc)
thread1.start()
thread2.start()
#pnp.move_to(left_joints, right_joints)
