# Copyright 2026
#
# Launch the ArUco pose reader from irn_g01.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use simulation clock'),
    DeclareLaunchArgument('image_topic',
                          default_value='/oakd/rgb/preview/image_raw/compressed',
                          description='RGB image topic used by aruco_reader'),
    DeclareLaunchArgument('compressed_image', default_value='true',
                          choices=['true', 'false'],
                          description='Use sensor_msgs/CompressedImage instead of Image'),
    DeclareLaunchArgument('camera_info_topic',
                          default_value='/oakd/rgb/preview/camera_info',
                          description='CameraInfo topic used for pose estimation'),
    DeclareLaunchArgument('pose_topic', default_value='/aruco/pose',
                          description='Output PoseStamped topic for detected marker'),
    DeclareLaunchArgument('camera_frame',
                          default_value='oakd_rgb_camera_optical_frame',
                          description='Frame id used in published marker poses'),
    DeclareLaunchArgument('marker_length', default_value='0.18',
                          description='Marker side length in meters'),
    DeclareLaunchArgument('aruco_id', default_value='372',
                          description='ArUco marker id to track'),
    DeclareLaunchArgument('aruco_dictionary', default_value='DICT_4X4_1000',
                          description='OpenCV ArUco dictionary name'),
    DeclareLaunchArgument('max_marker_lost_time', default_value='5.0',
                          description='Seconds before marker is considered lost'),
    DeclareLaunchArgument('max_marker_lost_frames', default_value='10',
                          description='Frames before marker is considered lost'),
    DeclareLaunchArgument('lost_condition', default_value='OR',
                          choices=['frame', 'time', 'AND', 'OR'],
                          description='Condition used to publish marker lost'),
    DeclareLaunchArgument('detect_frames_threshold', default_value='1',
                          description='Detected frames required before publishing poses'),
    DeclareLaunchArgument('output_rate_hz', default_value='5.0',
                          description='Maximum valid pose publish rate in Hz'),
    DeclareLaunchArgument('use_activation_mechanic_when_lost', default_value='true',
                          choices=['true', 'false'],
                          description='Require reactivation after marker is lost'),
    DeclareLaunchArgument('always_check_camera_parameters', default_value='false',
                          choices=['true', 'false'],
                          description='Keep listening to CameraInfo updates'),
]


def generate_launch_description():
    aruco_reader = Node(
        package='irn_g01',
        executable='aruco_reader',
        name='aruco_reader',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            {'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool)},
            {'image_topic': LaunchConfiguration('image_topic')},
            {'compressed_image': ParameterValue(
                LaunchConfiguration('compressed_image'), value_type=bool)},
            {'camera_info_topic': LaunchConfiguration('camera_info_topic')},
            {'pose_topic': LaunchConfiguration('pose_topic')},
            {'camera_frame': LaunchConfiguration('camera_frame')},
            {'marker_length': ParameterValue(
                LaunchConfiguration('marker_length'), value_type=float)},
            {'aruco_id': ParameterValue(
                LaunchConfiguration('aruco_id'), value_type=int)},
            {'aruco_dictionary': LaunchConfiguration('aruco_dictionary')},
            {'max_marker_lost_time': ParameterValue(
                LaunchConfiguration('max_marker_lost_time'), value_type=float)},
            {'max_marker_lost_frames': ParameterValue(
                LaunchConfiguration('max_marker_lost_frames'), value_type=int)},
            {'lost_condition': LaunchConfiguration('lost_condition')},
            {'detect_frames_threshold': ParameterValue(
                LaunchConfiguration('detect_frames_threshold'), value_type=int)},
            {'output_rate_hz': ParameterValue(
                LaunchConfiguration('output_rate_hz'), value_type=float)},
            {'use_activation_mechanic_when_lost': ParameterValue(
                LaunchConfiguration('use_activation_mechanic_when_lost'),
                value_type=bool)},
            {'always_check_camera_parameters': ParameterValue(
                LaunchConfiguration('always_check_camera_parameters'),
                value_type=bool)},
        ],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(aruco_reader)
    return ld
