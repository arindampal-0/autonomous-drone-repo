"""main file"""

import sys
import time

from pymavlink import mavutil


def print_delay(delay: int):
    """print delay"""
    if not isinstance(delay, int):
        print("'delay' parameter should be an integer.", file=sys.stderr)
        return

    if delay < 1:
        print("Delay should be an integer greater than 0.", file=sys.stderr)
        return

    for i in range(delay, -1, -1):
        print(f"Waiting for {i}", end="\r")
        time.sleep(1)
    print()


def main(args):
    """main file"""
    print("Pixhawk mavlink control")

    if not isinstance(args, list):
        print("'args' should be a list of string cli arguments.", file=sys.stderr)
        return 1

    if len(args) < 2:
        print("usage: python main.py [device_connection_string].", file=sys.stderr)
        return 1

    if not isinstance(args[1], str):
        print(
            "Second cli argument 'device_connection_string' should be a string.",
            file=sys.stderr,
        )
        return 1

    # connect to the pixhawk
    device_connection_string = args[1]
    master = mavutil.mavlink_connection(device_connection_string, baud=115200)

    # make sure the connection is valid
    master.wait_heartbeat()

    # change mode to 'STABILIZE'
    mode = "STABILIZE"

    if mode not in master.mode_mapping():
        print(f"Unknown mode: {mode}", file=sys.stderr)
        print("Try: ", list(master.mode_mapping().keys()), file=sys.stderr)
        return 1

    ## Get mode ID
    mode_id = master.mode_mapping()[mode]
    master.set_mode(mode_id)

    print("Waiting for mode change ACK...")
    while True:
        ack_msg = master.recv_match(type="COMMAND_ACK", blocking=True)

        ack_msg = ack_msg.to_dict()

        ## Waiting if acknowledged command is not `set_mode`
        if ack_msg["command"] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            continue

        ## Print ACK result
        print(mavutil.mavlink.enums["MAV_RESULT"][ack_msg["result"]].description)
        break
    print(f"Mode changed to {mode}")

    # Arm
    master.arducopter_arm()
    print("Arming motors.")

    print("Waiting for the vehicle to arm...")
    master.motors_armed_wait()
    print("Armed!")

    # Running the servo motors
    master.set_servo(1 + 8, 1100)
    print_delay(5)

    master.set_servo(1 + 8, 0)
    print_delay(3)

    # Disarm
    master.arducopter_disarm()
    print("Disarming motors.")

    print("Waiting for the vehicle to disarm...")
    master.motors_disarmed_wait()
    print("Motors disarmed!")

    master.close()
    print("Connection closed.")

    return 0


if __name__ == "__main__":
    sys.exit(main(args=sys.argv))
