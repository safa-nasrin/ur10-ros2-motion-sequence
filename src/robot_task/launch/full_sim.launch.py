import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Load MoveIt Configuration
    moveit_config = MoveItConfigsBuilder("ur10_with_gripper", package_name="robot_moveit_config").to_moveit_configs()
    
    # 2. Start Ignition Fortress
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py')]),
        launch_arguments={'ign_args': '-r empty.sdf'}.items(),
    )
    
    # 3. Spawn the Robot in Ignition
    spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'ur10_with_gripper'],
        output='screen'
    )    
    # 4. Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[moveit_config.robot_description]
    )
    
    # 5. Start MoveIt Move Group
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()]
    )
    
    # 6. Start RViz (Optional but required for visual proof)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ]
    )
    
    # 7. Load Controllers Sequentially (Required by assignment)
    load_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    load_arm = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_group_controller'],
        output='screen'
    )
    load_gripper = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gripper_controller'],
        output='screen'
    )
    
    # 8. Start your Python Task Node (Delayed slightly so controllers can boot first)
    task_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='robot_task',
                executable='motion_sequence_node',
                output='screen'
            )
        ]
    )

    # Event Handlers to ensure reliable startup order
    return LaunchDescription([
        gazebo,
        rsp,
        spawn_entity,
        move_group,
        rviz,
        # Start joint state broadcaster only after Gazebo spawns the robot
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_jsb]
            )
        ),
        # Start arm and gripper controllers only after broadcaster is ready
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_jsb,
                on_exit=[load_arm, load_gripper]
            )
        ),
        task_node
    ])
