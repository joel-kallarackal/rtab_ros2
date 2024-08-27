import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
import launch
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    parameters_rtab = {
        'frame_id':'zed_camera_link',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan': False,
        'icp_odometry': False,
        'visual_odometry': True,
        'subscribe_odom': True,
        'approx_sync': True,
        'use_action_for_goal': True,
    }
    
    remappings=[('scan','/scan'), ('/odom','/zed/zed_node/odom'), ('/rgb/image', '/zed/zed_node/rgb/image_rect_color'),('/camera/rgb/image_rect_color','/zed/zed_node/rgb/image_rect_color'),('rgb/camera_info','/zed/zed_node/rgb/camera_info'),('/depth/image','/zed/zed_node/depth/depth_registered')]
    
    include_rtabmap_ros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')),
        launch_arguments={
            "rtabmap_args": "--delete_db_on_start",
            "depth_topic": "/zed/zed_node/depth/depth_registered",
            "depth_camera_info_topic": "/zed/zed_node/depth/camera_info",
            "scan_cloud_topic": "/scan",
            "rgb_topic":"/zed/zed_node/rgb/image_rect_color",
            "camera_info_topic":"/zed/zed_node/rgb/camera_info",
            "frame_id": "zed_camera_link",
            "vo_frame_id": "odom",
            "subscribe_scan_cloud": "false",
            "wait_imu_to_init": "false",
            "imu_topic": "/zed/zed_node/imu/data",
            "rviz": "true",
            "publish_tf": "true",
            "use_sim_time": "false",
        }.items()
    )
    
    rtab_slam_node = Node(
         package='rtabmap_slam', executable='rtabmap', output='screen',parameters=[parameters_rtab], remappings=remappings,
         arguments=['-d'],
      )
    rtab_viz = Node(
        package = 'rtabmap_viz',executable='rtabmap_viz',output='screen',parameters=[parameters_rtab],remappings=remappings)
    
    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(rtab_slam_node)
    launchDescriptionObject.add_action(rtab_viz)

    return launchDescriptionObject