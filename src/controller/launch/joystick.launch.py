import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions




def generate_launch_description():
   joy_config = launch.substitutions.LaunchConfiguration("joy_config")
   joy_dev = launch.substitutions.LaunchConfiguration("joy_dev")
   config_filepath = launch.substitutions.LaunchConfiguration("config_filepath")


   return launch.LaunchDescription(
       [
           launch.actions.DeclareLaunchArgument("joy_config", default_value="joystick"),
           launch.actions.DeclareLaunchArgument(
               "joy_dev", default_value="/dev/input/js0"
           ),
           launch.actions.DeclareLaunchArgument(
               "config_filepath",
               default_value=[
                   launch.substitutions.TextSubstitution(
                       text=os.path.join(
                           get_package_share_directory("controller"), "config", ""
                       )
                   ),
                   joy_config,
                   launch.substitutions.TextSubstitution(text=".yaml"),
               ],
           ),
           launch_ros.actions.Node(
               package="joy",
               executable="joy_node",
               name="joy_node",
               parameters=[config_filepath],
           ),


           launch_ros.actions.Node(
               package="teleop_twist_joy",
               executable="teleop_node",
               name="teleop_twist_joy_node",
               parameters=[config_filepath],
           ),
          


       ]
   )
