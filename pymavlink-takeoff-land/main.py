"""main file"""

import sys
import time

from pymavlink import mavutil

def wait_for_ack(master, command) -> bool:
    """wait for ACK"""
    counter = 0
    while True:
        ack_msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
        ack_msg = ack_msg.to_dict()

        if counter > 5:
            return False

        if ack_msg["command"] != command:
            counter += 1
            continue

        print(mavutil.mavlink.enums["MAV_RESULT"][ack_msg["result"]].description)
        break

    return True


def main(args: list[str]) -> int:
    """main function"""
    print("pymavlink takeoff and land")

    if not isinstance(args, list):
        print("'args' parameter should be a list.", file=sys.stderr)
        return 1

    if len(args) != 2:
        print("usage: python main.py [device_connection_string]", file=sys.stderr)
        return 1
    
    if not isinstance(args[1], str):
        print("'args[1]' device_connection_string should be a string.", file=sys.stderr)
        return 1

    device_connection_string = args[1]

    # create a connection
    master = mavutil.mavlink_connection(device_connection_string, baud=115200)

    # make sure the connection is valid
    master.wait_heartbeat()

    # choose a mode
    mode = "STABILIZE"

    ## check if mode is available
    if mode not in master.mode_mapping():
        print(f"Unknown mode: {mode}", file=sys.stderr)
        print("Try: ", list(master.mode_mapping().keys()))
        return 1

    ## Get mode ID
    mode_id = master.mode_mapping()[mode]

    ## set mode message
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        0, mode_id, 0, 0, 0, 0, 0
    )
    
    ## Wait for ACK
    if not wait_for_ack(master, mavutil.mavlink.MAV_CMD_DO_SET_MODE):
        print("Could not set mode.", file=sys.stderr)
        return 1

    # arm drone
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0
    )

    if not wait_for_ack(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM):
        print("Could not arm the motors.", file=sys.stderr)
        return 1

    # takeoff to 1 meter
    takeoff_height = 1
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, takeoff_height
    )

    # TODO: wait for it to reach height

    time.sleep(10)

    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv))
