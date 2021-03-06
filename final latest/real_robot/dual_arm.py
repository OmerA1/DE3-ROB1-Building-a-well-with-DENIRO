#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import math
import time
import rospy
import rospkg
from utils import *
import numpy as np

import threading

from tf.transformations import *

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
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

class PickAndPlace(object):

    def __init__(self, limb, hover_distance = 0.15, verbose=True):

        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def move_to_start(self, start_angles=None):

        print("Moving the {0} arm to start pose...".format(self._limb_name))

        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
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



    def _guarded_move_to_joint_position(self, joint_angles):

        if joint_angles:

            self._limb.move_to_joint_positions(joint_angles)

        else:

            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")



    def gripper_open(self):

        self._gripper.open()

        rospy.sleep(1.0)



    def gripper_close(self):

        self._gripper.close()

        rospy.sleep(1.0)



    def _approach(self, pose):

        approach = copy.deepcopy(pose)

        # approach with a pose the hover-distance above the requested pose

        approach.position.z = approach.position.z + self._hover_distance

        joint_angles = self.ik_request(approach)

        self._guarded_move_to_joint_position(joint_angles)



    def _retract(self):

        # retrieve current pose from endpoint

        current_pose = self._limb.endpoint_pose()

        ik_pose = Pose()

        ik_pose.position.x = current_pose['position'].x

        ik_pose.position.y = current_pose['position'].y

        ik_pose.position.z = current_pose['position'].z + self._hover_distance

        ik_pose.orientation.x = current_pose['orientation'].x

        ik_pose.orientation.y = current_pose['orientation'].y

        ik_pose.orientation.z = current_pose['orientation'].z

        ik_pose.orientation.w = current_pose['orientation'].w

        joint_angles = self.ik_request(ik_pose)

        # servo up from current pose

        self._guarded_move_to_joint_position(joint_angles)



    def _servo_to_pose(self, pose):

        # servo down to release

        joint_angles = self.ik_request(pose)

        self._guarded_move_to_joint_position(joint_angles)

    def move(self, pose):
        self._approach(pose)
        self._servo_to_pose(pose)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        
        current_loc = self._limb.endpoint_pose()
        current_loc['position'].y = 0.1
        
        self._servo_to_pose(current_loc)
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        gripped = False
        gripcount = 0
        #print(self._gripper.position())
        while gripped == False:
            #print("Failed to grip, trying again...")
            gripcount += 1

            if self._gripper.position() < 4.5 and gripcount < 2:
                self.gripper_open()
                time.sleep(5)
                self.gripper_close()
                time.sleep(1)

            else:
                gripped = True

        self._retract()



    def place(self, pose):

        start = [0.75, 0, -0.17]
        end = [pose.position.x, pose.position.y, pose.position.z]

        path = plan_path(start, end)
        # servo above pose

        for i in range(1,len(path)):
            stop_point = Pose(position=Point(x=path[i,0],y=path[i,1],z=path[i,2]),orientation=pose.orientation)
            print(stop_point.position)
            self._servo_to_pose(stop_point)
        # servo to pose
        self._servo_to_pose(pose)
        #current_pose = self._limb.endpoint_pose()
        #if current_pose['position'].x != pose.position.x:
        #    print("repositioning")
        #    time.sleep(3)
        #    self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()



def load_gazebo_models(table_pose=Pose(position=Point(x=0.8, y=0.4, z=0.0)),

                       table_reference_frame="world",

                       brick_pose=Pose(position=Point(x=0.7, y=0, z=0.8210)),#,orientation=Quaternion(x=0.707072723701,y=0.0,z=0.0,w=0.707140837031)),

                       brick_reference_frame="world"):

    # Get Models' Path

    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"

    # Load Table SDF

    table_xml = ''

    with open (model_path + "cafe_table/model.sdf", "r") as table_file:

        table_xml=table_file.read().replace('\n', '')

    # Load Block URDF

    brick_xml = ''

    with open (model_path + "block/model.urdf", "r") as brick_file:

        brick_xml=brick_file.read().replace('\n', '')

    # Spawn Table SDF

    rospy.wait_for_service('/gazebo/spawn_sdf_model')

    try:

        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",

                             table_pose, table_reference_frame)

    except rospy.ServiceException, e:

        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    # Spawn Block URDF

    rospy.wait_for_service('/gazebo/spawn_urdf_model')

    try:

        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

        resp_urdf = spawn_urdf("brick", brick_xml, "/",

                               brick_pose, brick_reference_frame)

    except rospy.ServiceException, e:

        rospy.logerr("Spawn SDF service call failed: {0}".format(e))



def load_brick_l(idx,brick_pose=Pose(position=Point(x=0.5, y=-0.6, z=0.8210)),brick_reference_frame="world"):

    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"

    brick_xml = ''

    with open (model_path + "block/model.urdf", "r") as brick_file:

        brick_xml=brick_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')

    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

        resp_urdf = spawn_urdf("%s" %(idx), brick_xml, "/",

                               brick_pose, brick_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def load_brick_r(idx,brick_pose=Pose(position=Point(x=0.5, y=0.6, z=0.8210)),brick_reference_frame="world"):

    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"

    brick_xml = ''

    with open (model_path + "block/model.urdf", "r") as brick_file:

        brick_xml=brick_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')

    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

        resp_urdf = spawn_urdf("%s" %(idx), brick_xml, "/",

                               brick_pose, brick_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))



def load_table():

    table_pose=Pose(position=Point(x=0.8, y=0.0, z=0.0))
    table_pose_2=Pose(position=Point(x=0.8, y=0.913, z=0.0))
    table_pose_3=Pose(position=Point(x=0.8, y=-0.913, z=0.0))

    table_reference_frame="world"

    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"

    # Load Table SDF

    table_xml = ''

    with open (model_path + "cafe_table/model.sdf", "r") as table_file:

        table_xml=table_file.read().replace('\n', '')

    rospy.wait_for_service('/gazebo/spawn_sdf_model')

    try:

        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)

        resp_sdf = spawn_sdf("cafe_table_2", table_xml, "/",
                             table_pose_2, table_reference_frame)

        resp_sdf = spawn_sdf("cafe_table_3", table_xml, "/",
                             table_pose_3, table_reference_frame)

    except rospy.ServiceException, e:

        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

exitFlag = 0

class myThread (threading.Thread):

    def __init__(self, threadID, arm_name):
        super (myThread, self).__init__()
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.arm_name = arm_name

    def run(self):
        if self.arm_name == "left":
            master_l(spwan_loc, brick_l)
        if self.arm_name == "right":
            master_r(spwan_loc_r, brick_r)
        else:
            print("error")

def main():

    rospy.init_node("ik_pick_and_place_demo")

    #rospy.wait_for_message("/robot/sim/started", Empty)

    #load_table()
    #load_brick_l(-1)
    #load_brick_r(-2)

    left_arm = 'left'
    right_arm = 'right'

    hover_distance = 0.2# meters

    overhead_orientation = Quaternion(

                             x=-0.0249590815779,

                             y=0.999649402929,

                             z=0.00737916180073,

                             w=0.00486450832011)

    a,b,g = euler_from_quaternion([-0.0249590815779,0.999649402929,0.00737916180073,0.00486450832011])
    quat_r = quaternion_from_euler(a,-b,-g)
    overhead_orientation_r = Quaternion(x=quat_r[0],y=quat_r[1],z=quat_r[2],w=quat_r[3])

    global spwan_loc
    global spwan_loc_r
    global brick_l
    global brick_r

    y_offset = 0.015
    x_offset = 0.03

    spwan_loc = Pose(

        position=Point(x=0.5+x_offset, y=0.6+y_offset, z=-0.17),
        orientation= overhead_orientation)

    spwan_loc_r = Pose(

        position=Point(x=0.5+x_offset, y=-0.57, z=-0.17),
        orientation= overhead_orientation_r)

    starting_joint_angles_l = {'left_w0': 0.6699952259595108,

                         'left_w1': 1.030009435085784,

                         'left_w2': -0.4999997247485215,

                         'left_e0': -1.189968899785275,

                         'left_e1': 1.9400238130755056-0.3,

                         'left_s0': -0.08000397926829805+0.5,

                         'left_s1': -0.9999781166910306-0.5}

    starting_joint_angles_r = {'right_w0': 0,

                         'right_w1': 0,

                         'right_w2': 0,

                         'right_e0': 0,

                         'right_e1': 0,

                         'right_s0': -1.5,

                         'right_s1': -0.7}

    starting_joint_angles_r_2 = {'right_w0': 0.6699952259595108,

                 'right_w1': -1.030009435085784,

                 'right_w2': -0.4999997247485215,

                 'right_e0': 0.189968899785275,

                 'right_e1': 1.9400238130755056,

                 'right_s0': 0.08000397926829805-0.5,

                 'right_s1': -0.9999781166910306-0.5}


    brick_all = calculate_brick_locations_dual()
    brick_l = brick_all[0]
    brick_r = brick_all[1]

    global master_l
    global master_r
    pnp_r = PickAndPlace(right_arm, hover_distance)
    pnp_l = PickAndPlace(left_arm, hover_distance)
    pnp_r.move_to_start(starting_joint_angles_r)

    pnp_r.move_to_start(starting_joint_angles_r_2)
    pnp_l.move_to_start(starting_joint_angles_l)

    def master_r(spwan_loc_r,brick_r):
        for i in range(len(brick_r)):
            pose = Pose(
        position=Point(x=brick_r[i,0], y=brick_r[i,1], z=brick_r[i,2]),
        orientation=Quaternion(x=brick_r[i,3],y=brick_r[i,4],z=brick_r[i,5],w=brick_r[i,6]))
            pnp_r.pick(spwan_loc_r)
            time.sleep(0.5)
            pnp_r.place(pose)
            time.sleep(0.5)
            #load_brick_r(i)

    def master_l(spwan_loc,brick_l):
        for i in range(len(brick_l)):
            if i == 2:
                time.sleep(20)
            pose = Pose(
        position=Point(x=brick_l[i,0], y=brick_l[i,1], z=brick_l[i,2]),
        orientation=Quaternion(x=brick_l[i,3],y=brick_l[i,4],z=brick_l[i,5],w=brick_l[i,6]))
            pnp_l.pick(spwan_loc)
            time.sleep(0.5)
            pnp_l.place(pose)
            time.sleep(0.5)


            #load_brick_l(10*(i+1))

    thread1 = myThread(1, "left")
    thread2 = myThread(2, "right")

    #thread1.daemon = True
    #thread2.daemon = True

    thread1.start()
    thread2.start()

    return 0

global master_l
global master_r



if __name__ == '__main__':

    try:
        main()

    except KeyboardInterrupt:
        cleanupOnExit()
