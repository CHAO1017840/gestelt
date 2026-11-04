#!/usr/bin/env python3
"""Single-drone ROS 2 mission template."""

import gestelt_py_compat.rospy_compat as rospy
from geometry_msgs.msg import Transform
from gestelt_msgs.msg import Command, CommanderState, Goals


NUM_DRONES = 1
ROUTE = [(3.0, 2.0, 3.0), (5.0, 2.0, 3.0)]


def create_transform(x, y, z):
    transform = Transform()
    transform.translation.x = float(x)
    transform.translation.y = float(y)
    transform.translation.z = float(z)
    transform.rotation.w = 1.0
    return transform


def read_states():
    states = {}
    for drone_id in range(NUM_DRONES):
        states[drone_id] = rospy.wait_for_message(
            f"/drone{drone_id}/traj_server/state",
            CommanderState,
            timeout=5.0,
        )
    return states


def publish_command(command):
    for drone_id in range(NUM_DRONES):
        publisher = rospy.Publisher(
            f"/drone{drone_id}/traj_server/command",
            Command,
            queue_size=5,
        )
        message = Command()
        message.command = command
        publisher.publish(message)


def publish_goals(route):
    for drone_id in range(NUM_DRONES):
        publisher = rospy.Publisher(
            f"/drone{drone_id}/planner_adaptor/goals",
            Goals,
            queue_size=2,
        )
        message = Goals()
        message.header.frame_id = "world"
        message.transforms = [
            create_transform(x, y, z) for x, y, z in route
        ]
        publisher.publish(message)
    rospy.sleep(0.5)


def main():
    rospy.init_node("mission_startup", anonymous=True)
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        states = read_states()
        if all(state.traj_server_state == "MISSION" for state in states.values()):
            break
        if all(
            state.traj_server_state in ("HOVER", "MISSION")
            for state in states.values()
        ):
            publish_command(Command.MISSION)
        else:
            publish_command(Command.TAKEOFF)
        rate.sleep()

    print("Sending waypoints to UAVs")
    publish_goals(ROUTE)


if __name__ == "__main__":
    main()
