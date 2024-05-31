#!/usr/bin/env python3

import rospy 
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from movement_pipeline.cfg import JointMovementConfig 
import hsrb_interface


class JointMovement():
        
        def __init__(self):
            rospy.init_node("joint_movement_node")
    
            # Initialize the robot interface
            robot = hsrb_interface.Robot()
            self.whole_body = robot.get('whole_body')
            self.gripper = robot.get('gripper')
            self.tts = robot.get('default_tts')
            self.tts.language = self.tts.ENGLISH
            self.omni_base = robot.get('omni_base')

            # Initialize the old configuration with the default parameters
            self.old_config = {"ArmLiftJoint": 0.0, "ArmFlexJoint": 0.0, "ArmRollJoint": -1.57079632679, "WristFlexJoint": -1.57079632679, "WristRollJoint": 0.0, "HeadPanJoint": 0.0, "HeadTiltJoint": 0.0, "Gripper": 1.0}
            
            # Set up dynamic reconfigure server
            srv = Server(JointMovementConfig, self.reconfigure_callback)
            
            rospy.loginfo("Joint movement node initialized")

            rospy.spin()
    
        def reconfigure_callback(self, config, level):


            if config.Table == True:
                try:
                    self.omni_base.go_abs(0.35, 0.41, 0, 100.0) 
                    config = self.reset_config(config)
                except Exception as e:
                    rospy.logerr(f"Failed to move to table position. Error: {e}")
                    config.Table = False


            if config.Reset == True:
                config = self.reset_config(config)

            try:
                self.whole_body.move_to_joint_positions({
                    'arm_lift_joint': config.ArmLiftJoint,
                    'arm_flex_joint': config.ArmFlexJoint,
                    'arm_roll_joint': config.ArmRollJoint,
                    'wrist_flex_joint': config.WristFlexJoint,
                    'wrist_roll_joint': config.WristRollJoint,
                    'head_pan_joint': config.HeadPanJoint,
                    'head_tilt_joint': config.HeadTiltJoint
                })

                self.gripper.command(config.Gripper)

                if config.Line != "Default":
                    self.tts.say(config.Line)
                    config.Line = "Default"

                self.old_config = config


            except Exception as e:
                rospy.logerr("Failed to move to joint positions.")

            return self.old_config
        
        def reset_config(self, config):
            config.ArmLiftJoint = 0.0
            config.ArmFlexJoint = 0.0
            config.ArmRollJoint = -1.57079632679
            config.WristFlexJoint = -1.57079632679
            config.WristRollJoint = 0.0
            config.HeadPanJoint = 0.0
            config.HeadTiltJoint = 0.0
            config.Gripper = 1.0
            config.Table = False
            config.Reset = False
            config.Line = "Default"

            return config



if __name__ == "__main__":
    node = JointMovement()

