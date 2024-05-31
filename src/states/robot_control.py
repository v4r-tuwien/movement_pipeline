from math import pi
from tf.transformations import quaternion_about_axis 
import rospy
import actionlib
import smach
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Path, OccupancyGrid
from tmc_msgs.msg import Voice
from hsrb_interface import Robot
from grasping_pipeline_msgs.msg import BoundingBox3DStamped


class GoToNeutral(smach.State):
    """ Smach state that will move the robots joints to a
    predefined position, gazing at a fixed point.

    Outcomes:
        succeeded
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        # Robot initialization
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')

    def execute(self, userdata):
        rospy.loginfo('Executing state GoToNeutral')
        joint_positions = {
            'arm_flex_joint': 0, 
            'arm_lift_joint': 0, 
            'arm_roll_joint': pi/2, 
            'head_pan_joint': 0, 
            'head_tilt_joint': -0.675, 
            'wrist_flex_joint': -pi/2, 
            'wrist_roll_joint': 0
            }
        self.whole_body.move_to_joint_positions(joint_positions)
        return 'succeeded'

class MoveToJointPositions(smach.State):
    """ Smach state that will move the robots joints to the
    defined position.
    joints_positions_dict: dictionary of joint positions
    
    Outcomes:
        succeeded
    """

    def __init__(self, joint_positions_dict):
        smach.State.__init__(self, outcomes=['succeeded'])
        # Robot initialization
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.joint_positions_dict = joint_positions_dict

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveToJointPositions')
        self.whole_body.move_to_joint_positions(self.joint_positions_dict)
        return 'succeeded'


class GoBack(smach.State):
    """ Smach state that will move the robot backwards.
    Init: distance in meters
    Outcomes:
        succeeded
    """

    def __init__(self, distance):
        smach.State.__init__(self, outcomes=['succeeded'])
        # Robot initialization
        self.robot = Robot()
        self.base = self.robot.try_get('omni_base')
        self.whole_body = self.robot.try_get('whole_body')
        self.distance = distance

    def execute(self, userdata):
        rospy.loginfo('Executing state GoBack')
        self.base.go_rel(-self.distance, 0, 0)
        return 'succeeded'


class OpenGripper(smach.State):
    """ Opens the robots gripper

    Outcomes:
        succeeded
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

        # Robot initialization
        self.robot = Robot()
        self.gripper = self.robot.try_get('gripper')

    def execute(self, userdata):
        rospy.loginfo('Executing state OpenGripper')
        self.gripper.command(1.0)
        return 'succeeded'

class GoToWaypoint(smach.State):
    def __init__(self, x, y, phi_degree, frame_id='map', timeout=30.0):
        '''
        x, y, phi_degree: target pose
        frame_id: frame_id of the target pose
        timeout: timeout in seconds
        '''
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.move_client = actionlib.SimpleActionClient(
            '/move_base/move', MoveBaseAction)
        
        
        self.vel_sub = rospy.Subscriber('/base_velocity', Twist, self.vel_callback)
        self.path_sub = rospy.Subscriber('/base_local_path', Path, self.path_callback)
        self.map_sub = rospy.Subscriber('/dynamic_obstacle_map', OccupancyGrid, self.map_callback)

        self.curr_vel = None
        self.save_path = False
        self.curr_path = None
        self.map = None

        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.x = x
        self.y = y
        self.phi = phi_degree
        self.frame_id = frame_id
        self.timeout = timeout

    def vel_callback(self, msg):
        """Saves current velocity of the robot

        Args:
            msg (geometry_msgs.Twist): velocity of the robot 
        """
        self.curr_vel = msg

    def path_callback(self, msg):
        """Saves the current path of the robot

        Args:
            msg (nav_msgs.Path): Path of the robot
        """
        # TODO: Currently the path only gets saved after a new goal is published. 
        # This is just a workaround that doesn't take the current path changing (f.e. due to obstruction) into account.
        # Saving the path for every callback does sometimes lead to no path getting saved when the callback gets called when the HSR's path is occupied
        # Having a "if velocity != 0" doesn't solve this probleem all of the time due to unlucky timings.
        if self.save_path:
            self.curr_path = msg.poses
            self.save_path = False


    def map_callback(self, msg):
        """Saves the current map of the robot

        Args:
            msg (nav_msgs.OccupancyGrid): Map of the robot
        """
        self.map = msg

    def check_path(self, pose_stamped):
        """Checks if a point is occupied by checking the dynamic obstacle map

        Args:
            pose_stamped (geometry_msgs.PoseStamped): pose to check

        Returns:
            bool: True if the point is reachable (HSR is moving), False if not
        """
        
        print(f"Checking path for {pose_stamped.pose.position.x, pose_stamped.pose.position.y}")

        # Transformation from "Map" frame to the position of the dynamic obstacle map
        # f.e. if the HSR is in the map origin, the dynamic obstacle origin is at (-3.5, -3.5)
        # -> x = 0 in the map is x = 3.5 in the dynamic obstacle map -> x_obstacle_map = x_map - origin_obstacle
        x = pose_stamped.pose.position.x - self.map.info.origin.position.x
        y = pose_stamped.pose.position.y - self.map.info.origin.position.y

        # Calculation of the index of the point in the map
        idx_x = int(x / self.map.info.resolution)
        idx_y = int(y / self.map.info.resolution)

        # 1D array of the message is in row-major order
        idx_1D = idx_x + idx_y * self.map.info.width

        if idx_1D < 0 or idx_1D > (self.map.info.width * self.map.info.height):
            return False

        # Check if the point is occupied
        # -1 = Unknown (not visible for the HSR), 0 = Free (white in RViz), 100 = Occupied (black in RViz)
        prob = self.map.data[idx_1D]

        if prob == 0:
            return True
        
        return False



    def wait_for_result(self, og_move_goal, iteration=1):
        """Implementation of the actionlib.ActionClient.wait_for_result but with additional checking if the robot is not moving

        Args:
            iteration (int, optional): Prints the current iteration of wait_for_result 
                                       Every time wait_for_result gets called inside wait_for_result this gets increased by 1. 
                                       Defaults to 1. 

        Returns:
            bool: Returns the result of actionlib.ActienClient.get_result()
        """
        rospy.loginfo(f"Waiting for result of step {iteration}.")
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < self.timeout:
            rospy.sleep(1.0)

            state = self.move_client.get_state()

            if state != GoalStatus.ACTIVE:
                # State not active, get result
                rospy.loginfo(f"Finished step {iteration} with state {state}.")
                return self.move_client.get_result()

            if self.curr_vel.linear.x == 0 and self.curr_vel.linear.y == 0 and self.curr_vel.angular.x == 0 and self.curr_vel.angular.y == 0 and self.curr_vel.angular.z == 0:
                
                
                state = self.move_client.get_state()

                if state != GoalStatus.ACTIVE:
                    # For the rare case that the robot is not moving but the state is not active
                    rospy.loginfo(f"Finished step {iteration} with state {state}.")
                    return self.move_client.get_result()

                # Robot not moving (can falsely happen when he wants to get through open doors)
                rospy.logwarn("Robot not moving. There may be an obstacle in the way. Trying to find a new path.")
    
                path = self.curr_path
                finished_step = False
                for i, pose in enumerate(path[::-1]):
                    # Check if there is a reachable point on the path
                    if self.check_path(pose):

                        if i == 0:
                            # If the last point is reachable, the robot is already at the goal
                            rospy.loginfo("Robot is already at the goal")
                            finished_step = True
                            break

                        rospy.loginfo("Found new path")

                        # Moving to a new pose on the path
                        new_move_goal = MoveBaseGoal()
                        new_move_goal.target_pose = pose

                        self.move_client.send_goal(new_move_goal)
                        self.save_path = True

                        finished_step = self.wait_for_result(new_move_goal, i=iteration+1)
                        break

                
                if finished_step:
                    rospy.loginfo("Trying to get to older waypoint again.")
                    self.move_client.send_goal(og_move_goal)
                    self.save_path = True
                    return self.wait_for_result(og_move_goal, i=iteration)
                else:
                    rospy.logerr(f"Could not get to waypoint of step {iteration+1}.")
                    return False
                
        return False


    def execute(self, userdata):
        move_goal = MoveBaseGoal()
        
        move_goal.target_pose.header.frame_id = self.frame_id
        move_goal.target_pose.pose.position.x = self.x
        move_goal.target_pose.pose.position.y = self.y
        quat = quaternion_about_axis(self.phi * pi/180, (0, 0, 1))
        move_goal.target_pose.pose.orientation.x = quat[0]
        move_goal.target_pose.pose.orientation.y = quat[1]
        move_goal.target_pose.pose.orientation.z = quat[2]
        move_goal.target_pose.pose.orientation.w = quat[3]
        
        rospy.loginfo("Waiting for move client server!")
        finished = self.move_client.wait_for_server(rospy.Duration(self.timeout))
        
        if finished:
            self.move_client.send_goal(move_goal)
            self.save_path = True

            while self.curr_path is None and self.curr_vel is None:
                rospy.sleep(0.1)

            finished = self.wait_for_result(move_goal)

            if finished:
                # wait for robot to settle down
                rospy.sleep(1.0)
                return 'succeeded'
            else:
                # Was not able to get to the original waypoint. 
                # Outcome is still "succeeeded" because the statemachien would change to the same state that reaching the waypoint would.
                rospy.logerr("Could not get to waypoint of step 1.")
                self.move_client.cancel_all_goals()
                return 'succeeded'
        else:
            rospy.logerr("Could not connect to move server!")
            return 'aborted'
        
    
class GoToAndLookAtPlacementArea(smach.State):
    def __init__(self, outcomes=['succeeded', 'aborted'], input_keys=['grasp_object_name'], output_keys=['placement_area_bb'] ):
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
        self.move_client = actionlib.SimpleActionClient(
            '/move_base/move', MoveBaseAction)
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.timeout = 40.0
    
    def execute(self, userdata):
        placement_planes = rospy.get_param('/placement_planes')
        placement_objects = rospy.get_param('/placement_objects')
        grasp_object_name = userdata.grasp_object_name
        
        placement_plane_name = placement_objects[grasp_object_name]
        placement_plane_config = placement_planes['planes'][placement_plane_name]
        frame_id = placement_planes['metadata']['frame_id']
        waypoint = placement_plane_config['waypoint']
        placement_area = placement_plane_config['placement_area']
        placement_area_bb = BoundingBox3DStamped()
        placement_area_bb.center.position.x = placement_area['center'][0]
        placement_area_bb.center.position.y = placement_area['center'][1]
        placement_area_bb.center.position.z = placement_area['center'][2]
        placement_area_bb.center.orientation.w = 1.0
        placement_area_bb.size.x = placement_area['size'][0]
        placement_area_bb.size.y = placement_area['size'][1]
        placement_area_bb.size.z = placement_area['size'][2]
        placement_area_bb.header.frame_id = frame_id
        userdata.placement_area_bb = placement_area_bb
        
        move_goal = MoveBaseGoal()
        
        move_goal.target_pose.header.frame_id = frame_id
        move_goal.target_pose.pose.position.x = waypoint[0]
        move_goal.target_pose.pose.position.y = waypoint[1]
        quat = quaternion_about_axis(waypoint[2] * pi/180, (0, 0, 1))
        move_goal.target_pose.pose.orientation.x = quat[0]
        move_goal.target_pose.pose.orientation.y = quat[1]
        move_goal.target_pose.pose.orientation.z = quat[2]
        move_goal.target_pose.pose.orientation.w = quat[3]

        # arm_lift moves by ~ 10 cm for each 0.2 increase and has to be between 0 and 0.6
        arm_lift_joint = max(0.0, (placement_area['center'][2] - 0.45)*2.0)
        rospy.logerr(f"{arm_lift_joint = }")
        rospy.logerr(f"{placement_area['center'][2] = }")
        arm_lift_joint = min(arm_lift_joint, 0.6)

        # move that pesky arm out of the way
        arm_flex_joint = -2.6 if arm_lift_joint > 0.1 else -0.1
        self.whole_body.move_to_joint_positions({'arm_flex_joint': arm_flex_joint, 'arm_lift_joint': arm_lift_joint})
        rospy.sleep(5.0)
        
        rospy.loginfo("Waiting for move client server!")
        finished = self.move_client.wait_for_server(rospy.Duration(self.timeout))
        
        if finished:
            self.move_client.send_goal(move_goal)
            finished = self.move_client.wait_for_result(rospy.Duration(self.timeout))

            if finished:
                # wait for robot to settle down
                rospy.sleep(1.0)
                return 'succeeded'
            else:
                rospy.logerr("Move server execution timed out!")
                self.move_client.cancel_all_goals()
                return 'aborted'
        else:
            rospy.logerr("Could not connect to move server!")
            return 'aborted'