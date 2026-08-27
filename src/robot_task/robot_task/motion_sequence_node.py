import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class MotionSequenceNode(Node):
    def __init__(self):
        super().__init__('motion_sequence_node')
        self.get_logger().info('Motion Sequence Node Initialized. Starting sequence...')
        
        # Parameters for dwell time and looping
        self.declare_parameter('dwell_time', 2.0)
        self.declare_parameter('enable_loop', True)
        
        self.dwell_time = self.get_parameter('dwell_time').value
        self.enable_loop = self.get_parameter('enable_loop').value
        
        # Action clients for controllers
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_group_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        
        # Wait for action servers
        self.get_logger().info('Waiting for controller action servers...')
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info('Controller action servers connected!')
        
        # Define named poses in radians
        # Order: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
        self.poses = {
         "Home": [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],  # Standing straight up vertical
         "Pose 1": [0.5, -1.0, 1.2, -0.5, 1.5, 0.0],
         "Pose 2": [-0.5, -1.2, 1.0, 0.5, -1.5, 0.0]
        }
        
        # Gripper positions: [left_finger, right_finger]
        self.gripper_open = [0.03, 0.03]

        # Start the sequence
        self.execute_sequence()

    def send_arm_goal(self, positions):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 3
        goal_msg.trajectory.points = [point]
        
        send_goal_future = self.arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            return False
            
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

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
    node = MotionSequenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down cleanly via Keyboard Interrupt (Ctrl+C).')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
