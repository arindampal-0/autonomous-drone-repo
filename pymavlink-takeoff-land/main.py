"""main file"""

import sys

from pymavlink import mavutil

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
    master.mav.set_mode_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        0, mode_id, 0, 0, 0, 0, 0)
    
    ## Wait for ACK
    while True:
        ack_msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
        ack_msg = ack_msg.to_dict()

        if ack_msg["command"] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            continue

        print(mavutil.mavlink.enums["MAV_RESULT"][ack_msg["result"]].description)
        break

    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv))
