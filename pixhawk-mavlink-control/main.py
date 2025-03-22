"""main file"""

import sys

from pymavlink import mavutil


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

    # Arm
    master.arducopter_arm()
    print("Arming motors.")

    print("Waiting for the vehicle to arm...")
    master.motor_armed_wait()
    print("Armed!")

    # Disarm
    master.arducopter_disarm()
    print("Disarming motors.")

    print("Waiting for the vehicle to disarm...")
    master.motor_disarmed_wait()
    print("Motors disarmed!")

    return 0


if __name__ == "__main__":
    sys.exit(main(args=sys.argv))
