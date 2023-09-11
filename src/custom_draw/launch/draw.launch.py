from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    shape_node = Node(
        package='custom_draw',
        executable='shape_node',
        name='shape_node',
        output='screen',
        arguments=['--wait-for-input']
    )
    
    turtle_commander = Node(
        package='custom_draw',
        executable='turtle_commander',
        name='turtle_commander',
        output='screen'
    )

   

  

    
    ld.add_action(turtle_commander)
    ld.add_action(shape_node)

    return ld