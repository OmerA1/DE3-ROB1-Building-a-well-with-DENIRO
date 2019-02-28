    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        #self._gripper.command_position(1)

        #check if the brick has been gripped, if not, repeat grip process
        gripped = False
        while gripped == False:
            if self._gripper.position() < 55:
                self.gripper_open()
                rospy.sleep(3)
                self.gripper_close()
                rospy.sleep(1)
            else:
                gripped = True
