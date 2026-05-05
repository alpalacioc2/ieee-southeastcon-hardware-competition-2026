from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess


def generate_launch_description():

    # Camera node
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_node',
        output='screen'
    )

    # Set camera to manual exposure
    set_camera_manual = ExecuteProcess(
        cmd=['v4l2-ctl', '-d', '/dev/video0', '-c', 'auto_exposure=1'],
        output='screen'
    )

    # Set camera exposure time
    set_camera_exposure = ExecuteProcess(
        cmd=['v4l2-ctl', '-d', '/dev/video0', '-c', 'exposure_time_absolute=25'],
        output='screen'
    )

    # Lidar driver launch
    lidar_driver = ExecuteProcess(
        cmd=['ros2', 'launch', 'sllidar_ros2', 'sllidar_a1_launch.py'],
        output='screen'
    )

    # RGB color detector node
    image_color_detector = Node(
        package='rgb_color_detector',
        executable='image_color_detector_node',
        name='image_color_detector_node',
        output='screen'
    )

    # Drone node
    drone_node = Node(
        package='launch_drone_pkg',
        executable='launch_drone_node',
        name='launch_drone_node',
        output='screen'
    )

    # Convert lidar scan -> 12 sensors
    lidar_twelve = Node(
        package='lidar_wall_follow_v1',
        executable='lidar_twelve_sensors',
        name='lidar_twelve_sensors',
        output='screen'
    )

    # Wall follower node
    wall_follower = Node(
        package='lidar_wall_follow_v1',
        executable='wall_follow_node',
        name='wall_follow_node',
        output='screen'
    )

    # Xiao IR bridge
    xiao_bridge = Node(
        package='xiao_ir_pkg',
        executable='send_ir_node',
        name='send_ir_node',
        output='screen'
    )

    # Main controller
    program_controller = Node(
        package='lidar_wall_follow_v1',
        executable='program_controller_node',
        name='program_controller_node',
        output='screen'
    )

    return LaunchDescription([

        # Start camera immediately
        camera_node,

        # Enable manual exposure
        TimerAction(
            period=5.0,
            actions=[set_camera_manual]
        ),

        # Set exposure value
        TimerAction(
            period=5.0,
            actions=[set_camera_exposure]
        ),

        # Start lidar driver after 2 seconds
        TimerAction(
            period=2.0,
            actions=[lidar_driver]
        ),

        # Start color detector after 2 seconds
        TimerAction(
            period=2.0,
            actions=[image_color_detector]
        ),

        # Start drone node after 5 seconds
        TimerAction(
            period=5.0,
            actions=[drone_node]
        ),

        # Start lidar -> sensor converter
        TimerAction(
            period=6.0,
            actions=[lidar_twelve]
        ),

        # Start wall follower
        TimerAction(
            period=8.0,
            actions=[wall_follower]
        ),

        # Start IR bridge
        TimerAction(
            period=10.0,
            actions=[xiao_bridge]
        ),

        # Start main controller
        TimerAction(
            period=10.0,
            actions=[program_controller]
        ),
    ])