from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Find the package where the launch files are located
    realsense2_camera_path = FindPackageShare('realsense2_camera')

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([realsense2_camera_path, '/launch/rs_launch.py']),
        launch_arguments = {
            'camera_name':                      'camera',
            'camera_namespace':                 'camera',
            'accelerate_gpu_with_glsl':         'false',
            'output':                           'screen',
            'enable_color':                     'true',
            'enable_depth':                     'true',
            'depth_module.enable_auto_exposure':'true',
            'enable_sync':                      'false',
            'clip_distance':                    '-2.0',
            'publish_tf':                       'true',
            'tf_publish_rate':                  '14.0',
            'pointcloud.enable':                'true',
            'pointcloud.stream_filter':         '2',
            'pointcloud.stream_index_filter':   '0',
            'pointcloud.ordered_pc':            'false',
            'align_depth.enable':               'true',
            'colorizer.enable':                 'false',
            'decimation_filter.enable':         'true',
            'spatial_filter.enable':            'true',
            'temporal_filter.enable':           'true',
            'disparity_filter.enable':          'false',
            'hole_filling_filter.enable':       'false',
            'hdr_merge.enable':                 'false',
            'wait_for_device_timeout':          '-1.0',
            'reconnect_timeout':                '6.0',
        }.items(),
    )

    my_robot_package_path = FindPackageShare('simple_object_detection')

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            my_robot_package_path, 'config', 'qr_code_display.rviz'
        ])]
    )

    # Bring up the node to detect the qr code
    qr_code_detector_node = Node(
        name="qr_code_detector",
        package="simple_object_detection",
        executable="qr_code_detector"
    )

    launched_nodes = LaunchDescription([
        realsense_node,
        rviz_node,
        qr_code_detector_node
    ])

    return launched_nodes