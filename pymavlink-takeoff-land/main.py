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

    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv))
