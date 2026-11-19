#!/usr/bin/env python3
"""Take off and hold HOVER without publishing navigation goals."""

import sys
import time

import gestelt_py_compat.rospy_compat as rospy
from gestelt_msgs.msg import Command, CommanderState


DRONE_ID = 0


def get_state(timeout=5.0):
    return rospy.wait_for_message(
        f"/drone{DRONE_ID}/traj_server/state",
        CommanderState,
        timeout=timeout,
    )


def main():
    rospy.init_node("hover_only_test", anonymous=True)
    publisher = rospy.Publisher(
        f"/drone{DRONE_ID}/traj_server/command",
        Command,
        queue_size=5,
    )
    rate = rospy.Rate(5)

    command = Command()
    command.command = Command.TAKEOFF

    last_state = None
    last_status_time = 0.0
    while not rospy.is_shutdown():
        now = time.monotonic()
        try:
            state = get_state()
        except TimeoutError:
            if now - last_status_time >= 5.0:
                print(
                    "[hover-only] waiting for /drone0/traj_server/state",
                    flush=True,
                )
                last_status_time = now
            continue
        if state.traj_server_state != last_state or now - last_status_time >= 5.0:
            print(
                f"[hover-only] traj_server={state.traj_server_state} "
                f"armed={state.armed}"
            )
            last_state = state.traj_server_state
            last_status_time = now
        if state.traj_server_state in ("HOVER", "MISSION"):
            break
        publisher.publish(command)
        rate.sleep()

    print("[hover-only] HOVER reached; no navigation goals will be sent")
    return 0


if __name__ == "__main__":
    sys.exit(main())
