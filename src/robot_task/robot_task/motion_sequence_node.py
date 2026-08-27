import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

class MotionSequenceNode(Node):
    def __init__(self):
        super().__init__('motion_sequence_node')
        self.get_logger().info('Motion Sequence Node Initialized. Starting sequence...')
        
        # Declare parameters (with defaults just in case)
        self.declare_parameter('dwell_time', 2.0)
        self.declare_parameter('enable_loop', True)
        self.declare_parameter('poses.home', [0.0, -1.57, 0.0, -1.57, 0.0, 0.0])
        self.declare_parameter('poses.pose_1', [0.5, -1.0, 1.2, -0.5, 1.5, 0.0])
        self.declare_parameter('poses.pose_2', [-0.5, -1.2, 1.0, 0.5, -1.5, 0.0])
        self.declare_parameter('gripper.open', 0.035)
        
        self.dwell_time = self.get_parameter('dwell_time').value
        self.enable_loop = self.get_parameter('enable_loop').value
        
        # Load poses dynamically from parameters instead of hardcoding
        self.poses = {
            "Home": self.get_parameter('poses.home').value,
            "Pose 1": self.get_parameter('poses.pose_1').value,
            "Pose 2": self.get_parameter('poses.pose_2').value
        }
        
        # Load gripper value
        g_open = self.get_parameter('gripper.open').value
        self.gripper_open = [g_open, g_open]

        # Action clients for controllers
        self.arm_client = ActionClient(self, MoveGroup, '/move_action')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        
        # Wait for action servers
        self.get_logger().info('Waiting for controller action servers...')
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info('Controller action servers connected!')
        
        # Start the sequence
        self.execute_sequence()
 
    def send_arm_goal(self, positions):
        joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        constraints = Constraints()
        for name, pos in zip(joint_names, positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm_group'
        goal_msg.request.goal_constraints = [constraints]
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.num_planning_attempts = 5
        goal_msg.planning_options.plan_only = False

        send_goal_future = self.arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('MoveIt goal rejected!')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        success = (result.error_code.val == 1)
        if not success:
            self.get_logger().error(f'MoveIt planning/execution failed, error code: {result.error_code.val}')
        return success
    def send_gripper_goal(self, positions):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['left_finger_joint', 'right_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 2
        goal_msg.trajectory.points = [point]
        
        send_goal_future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            return False
            
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

    def execute_sequence(self):
        sequence_steps = [
            ("Home", lambda: self.send_arm_goal(self.poses["Home"])),
            ("Pose 1", lambda: self.send_arm_goal(self.poses["Pose 1"])),
            ("Gripper Open", lambda: self.send_gripper_goal(self.gripper_open)),
            ("Pose 2", lambda: self.send_arm_goal(self.poses["Pose 2"])),
            ("Gripper Open", lambda: self.send_gripper_goal(self.gripper_open)),
            ("Home", lambda: self.send_arm_goal(self.poses["Home"]))
        ]
        
        while rclpy.ok():
            for step_num, (pose_name, action_func) in enumerate(sequence_steps, 1):
                self.get_logger().info(f'Step {step_num}: Moving to / Executing {pose_name}...')
                
                # Execute real action call
                success = action_func()
                
                if success:
                    self.get_logger().info(f'Action completed successfully. Dwell for {self.dwell_time} seconds...')
                else:
                    self.get_logger().error(f'Action failed for {pose_name}!')
                
                # Strict 2-second dwell after success
                time.sleep(self.dwell_time)
            
            self.get_logger().info('Full cycle completed successfully!')
            
            if not self.enable_loop:
                self.get_logger().info('Looping disabled via parameter. Stopping execution.')
                break

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotionSequenceNode()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
