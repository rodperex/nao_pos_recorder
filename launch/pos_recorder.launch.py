from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    
    # Arguments
    joint_states_topic_arg = DeclareLaunchArgument(
        'joint_states_topic',
        default_value='/joint_states',
        description='Topic name for joint states'
    )
    
    stiffness_topic_arg = DeclareLaunchArgument(
        'stiffness_topic',
        default_value='/effectors/joint_stiffnesses',
        description='Topic name for joint stiffness control'
    )
    
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=os.path.expanduser('~/nao_recordings'),
        description='Directory where .pos files will be saved'
    )
    
    default_duration_arg = DeclareLaunchArgument(
        'default_duration',
        default_value='1000',
        description='Default duration in milliseconds for keyframes'
    )
    
    movement_type_arg = DeclareLaunchArgument(
        'movement_type',
        default_value='arms',
        description='Type of movement to record: "arms" or "legs"'
    )
    
    # Node
    pos_recorder_node = Node(
        package='nao_pos_recorder',
        executable='pos_recorder',
        name='pos_recorder_node',
        output='screen',
        parameters=[{
            'joint_states_topic': LaunchConfiguration('joint_states_topic'),
            'stiffness_topic': LaunchConfiguration('stiffness_topic'),
            'output_dir': LaunchConfiguration('output_dir'),
            'default_duration': LaunchConfiguration('default_duration'),
            'movement_type': LaunchConfiguration('movement_type'),
        }]
    )
    
    return LaunchDescription([
        joint_states_topic_arg,
        stiffness_topic_arg,
        output_dir_arg,
        default_duration_arg,
        movement_type_arg,
        pos_recorder_node,
    ])
