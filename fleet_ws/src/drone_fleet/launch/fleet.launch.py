from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    drones = [
        ("Alpha", 100.0),
        ("Beta", 60.0),
        ("Gamma", 35.0),
    ]

    actions = []
    for drone_name, battery in drones:
        actions.append(
            Node(
                package="drone_fleet",
                executable="drone_node",
                name=f"{drone_name.lower()}_drone_node",
                output="screen",
                parameters=[
                    {"drone_name": drone_name, "initial_battery": battery, "mission_name": f"{drone_name} Mission"},
                ],
            )
        )

    actions.append(
        Node(
            package="drone_fleet",
            executable="fleet_manager",
            name="fleet_manager",
            output="screen",
        )
    )

    actions.append(
        Node(
            package="drone_fleet",
            executable="health_monitor",
            name="health_monitor",
            output="screen",
        )
    )

    return LaunchDescription(actions)