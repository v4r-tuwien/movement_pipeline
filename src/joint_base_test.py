#!/usr/bin/env python3

# Test file to test joint/base movement without the hsrb import

# whole_body.move_to_joint_positions mit '/hsrb/arm_trajectory_controller/command' (Publisher oder Action Server)
# whole_body.joint_positions mit '/hsrb/arm_trajectory_controller/state' (Subscriber)
# Analog für Head siehe Tutorial
# omni_base.go_rel mit '/hsrb/omni_base_controller/command' (Publisher oder Action Server)
# gripper.command mit '/hsrb/gripper_controller/command' (Publisher)
# geht auch mit Action Server
# gripper.apply_force mit /hsrb/gripper_controller/apply_force/ (Action Server)
# tts.say mit topic /talk_request und message tmc_msgs/Voice

"""
import controller_manager_msgs.srv
import rospy
import trajectory_msgs.msg

rospy.init_node('test')

# initialize ROS publisher
pub = rospy.Publisher('/hsrb/arm_trajectory_controller/command',
                      trajectory_msgs.msg.JointTrajectory, queue_size=10)

# wait to establish connection between the controller
while pub.get_num_connections() == 0:
    rospy.sleep(0.1)

# make sure the controller is running
rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
list_controllers = (
    rospy.ServiceProxy('/hsrb/controller_manager/list_controllers',
                       controller_manager_msgs.srv.ListControllers))
running = False
while running is False:
    rospy.sleep(0.1)
    for c in list_controllers().controller:
        # Important existing controller:
        # joint_state_controller
        # omni_base_controller
        # head_trajectory_controller
        # arm_trajectory_controller
        # gripper_controller

        if c.name == 'arm_trajectory_controller' and c.state == 'running':
            running = True

# fill ROS message
traj = trajectory_msgs.msg.JointTrajectory()
traj.joint_names = ["arm_lift_joint", "arm_flex_joint",
                    "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
p = trajectory_msgs.msg.JointTrajectoryPoint()
p.positions = [0.2, -0.5, 0, 0, 0]
p.velocities = [0, 0, 0, 0, 0]
p.time_from_start = rospy.Duration(3)
traj.points = [p]

# publish ROS message
pub.publish(traj)
"""

"""
import controller_manager_msgs.srv
import rospy
import trajectory_msgs.msg

rospy.init_node('test')

# initialize ROS publisher
pub = rospy.Publisher(
    '/hsrb/omni_base_controller/command',
    trajectory_msgs.msg.JointTrajectory, queue_size=10)

# wait to establish connection between the controller
while pub.get_num_connections() == 0:
    rospy.sleep(0.1)

# make sure the controller is running
rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
list_controllers = rospy.ServiceProxy(
    '/hsrb/controller_manager/list_controllers',
    controller_manager_msgs.srv.ListControllers)
running = False
while running is False:
    rospy.sleep(0.1)
    for c in list_controllers().controller:
        if c.name == 'omni_base_controller' and c.state == 'running':
            running = True

# fill ROS message
traj = trajectory_msgs.msg.JointTrajectory()
traj.joint_names = ["odom_x", "odom_y", "odom_t"]
p = trajectory_msgs.msg.JointTrajectoryPoint()
p.positions = [-1, -1, 0]
p.velocities = [0, 0, 0]
p.time_from_start = rospy.Duration(15)
traj.points = [p]

# publish ROS message
pub.publish(traj)
"""

"""
import controller_manager_msgs.srv
import rospy
import trajectory_msgs.msg

rospy.init_node('test')

# initialize ROS publisher
pub = rospy.Publisher(
    '/hsrb/gripper_controller/command',
    trajectory_msgs.msg.JointTrajectory, queue_size=10)

# wait to establish connection between the controller
while pub.get_num_connections() == 0:
    rospy.sleep(0.1)

# make sure the controller is running
rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
list_controllers = rospy.ServiceProxy(
    '/hsrb/controller_manager/list_controllers',
    controller_manager_msgs.srv.ListControllers)
running = False
while running is False:
    rospy.sleep(0.1)
    for c in list_controllers().controller:
        if c.name == 'gripper_controller' and c.state == 'running':
            running = True

# fill ROS message
traj = trajectory_msgs.msg.JointTrajectory()
traj.joint_names = ["hand_motor_joint"]
p = trajectory_msgs.msg.JointTrajectoryPoint()
p.positions = [1.2]
p.velocities = [0]
p.effort = [0.1]
p.time_from_start = rospy.Duration(3)
traj.points = [p]

# publish ROS message
pub.publish(traj)
"""

from tmc_msgs.msg import Voice
import rospy

rospy.init_node("test_node")

pub = rospy.Publisher('/talk_request', Voice, queue_size=0)
rospy.sleep(1)

msg = Voice()
msg.interrupting = False
msg.queueing = False
msg.language = 1
msg.sentence = "Hello, I am Sasha!"

try:
    pub.publish(msg)
except Exception as e:
    print(e)
print("DONE")
