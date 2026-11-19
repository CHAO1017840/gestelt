#!/usr/bin/env python3

import gestelt_py_compat.rospy_compat as rospy
from gestelt_msgs.msg import Command, CommanderState, Goals
from geometry_msgs.msg import Transform


num_drones = 1
goal_publishers = [
    rospy.Publisher(
        f"/drone{i}/planner_adaptor/goals",
        Goals,
        queue_size=2,
    )
    for i in range(num_drones)
]
server_states = {}


def get_server_state_callback():
    for drone_id in range(num_drones):
        msg = rospy.wait_for_message(
            f"/drone{drone_id}/traj_server/state",
            CommanderState,
            timeout=5.0,
        )
        server_states[str(msg.drone_id)] = msg


def check_traj_server_states(desired_state):
    get_server_state_callback()
    if not server_states:
        print("No server states received")
        return False
    return all(
        state.traj_server_state == desired_state
        for state in server_states.values()
    )


def publish_server_cmd(event_enum):
    for drone_id in range(num_drones):
        msg = Command()
        msg.command = event_enum
        publisher = rospy.Publisher(
            f"/drone{drone_id}/traj_server/command",
            Command,
            queue_size=5,
        )
        publisher.publish(msg)


def create_transform(x, y, z):
    transform = Transform()
    transform.translation.x = x
    transform.translation.y = y
    transform.translation.z = z
    transform.rotation.w = 1.0
    return transform


def publish_goals(goals):
    if len(goals) != num_drones:
        raise ValueError(
            f"Expected {num_drones} goal lists, got {len(goals)}"
        )
    for index, goal in enumerate(goals):
        msg = Goals()
        msg.header.frame_id = "world"
        msg.transforms = goal
        goal_publishers[index].publish(msg)
    rospy.sleep(0.5)


def main():
    rospy.init_node("mission_startup_rmuc2025", anonymous=True)
    rate = rospy.Rate(5)

    if not check_traj_server_states("MISSION"):
        print("Setting to HOVER mode")
        while not rospy.is_shutdown():
            if check_traj_server_states("HOVER") or check_traj_server_states(
                "MISSION"
            ):
                break
            publish_server_cmd(0)
            rate.sleep()

        print("Setting to MISSION mode")
        while not rospy.is_shutdown():
            if check_traj_server_states("MISSION"):
                break
            publish_server_cmd(2)
            rate.sleep()

    print("Sending RMUC 2025 goal")
    # The 2025 national map was shifted by (+3, -12) so the free start point
    # (-3, 12) in the source map is the PX4 local origin (0, 0).
    publish_goals([
        [create_transform(6.0, -24.0, 1.0)]
    ])


if __name__ == "__main__":
    main()
