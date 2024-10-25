import launch
import launch_ros.actions
import os

from ament_index_python.packages import get_package_share_directory


from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_launch = get_package_share_directory('py_launch')

    world_sdf_path = os.path.join(pkg_launch, 'worlds', 'world.sdf')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_sdf_path}'}.items(),
    )


    return launch.LaunchDescription([
        gz_sim,
        launch_ros.actions.Node(
            package='py_robot_controller',
            executable='controller',
            prefix='gnome-terminal -- ',
            name='vehicle_controller'),
        launch_ros.actions.Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
        ),
        launch_ros.actions.Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'],
        ),
  ])