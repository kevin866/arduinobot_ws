from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    arduinobot_description = get_package_share_directory("arduinobot_description")
    arduinobot_description_prefix = get_package_prefix("arduinobot_description")

    model_path = os.path.join(arduinobot_description, "models")
    model_path += pathsep + os.path.join(arduinobot_description_prefix, "share")

    # env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH",  model_path)

    env_variable = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            model_path)


    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value = os.path.join(arduinobot_description, 'urdf', 'arduinobot.urdf.xacro'),
        description='Absolute path to the robot urdf file'
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), 
                                       value_type=str)

  
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{"robot_description": robot_description}],
        output='screen'
    )

    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = os.path.join(
        arduinobot_description,
        'worlds',
        'empty_world.world'
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        )
        ,
        launch_arguments={'gz_args': ['-r -s -v1 ']}.items()
    )
    # start_gazebo_client = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
    #     ),
    #     launch_arguments={'gz_args': '-g -v1 '}.items()
    # )


    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='create',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-entity', 'arduinobot', '-topic', 'robot_description']
    )
    

    return LaunchDescription([
        env_variable,
        model_arg,
        robot_state_publisher_node,
        # gz_sim,
        start_gazebo_server,
        # start_gazebo_client,
        spawn_robot
    ])
        
    