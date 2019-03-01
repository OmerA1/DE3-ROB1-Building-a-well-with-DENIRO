import sys
import rospy
from std_msgs.msg import (
    UInt16,
)
import baxter_interface
from baxter_interface import CHECK_VERSION

class PickandPlace(object):
	def __init__(self, pose, hover_distance = 0.1, verbose=True):
		#initialise arms and grippers
		self._left_arm = baxter_interface.Limb("left")
		self._right_arm = baxter_interface.Limb("right")
		self._left_gripper = baxter_interface.Gripper("left")
		self._right_gripper = baxter_interface.Gripper("right")
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
	
	def move_to(self, left_joint_positions, right_joint_positions):
		rate = rospy.Rate(100)
		if left_joint_positions and right_joint_positions:
			self._left_arm.move_to_joint_positions(left_joint_positions)
			self._right_arm.move_to_joint_positions(right_joint_positions)
		else:
			rospy.logerr("No Joint Angles, staying put."
	
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
