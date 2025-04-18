"""main file"""

import sys
import time
from enum import Enum

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


def print_available_modes(master: mavutil.mavserial):
    """print available modes"""
    if not isinstance(master, mavutil.mavserial):
        print("'master' should be an instance of 'mavutil.mavserail'.", file=sys.stderr)
        return

    print("Flight Modes: ", list(master.mode_mapping().keys()))


class FlightMode(str, Enum):
    """flight mode"""

    STABILIZE = "STABILIZE"
    ALT_HOLD = "ALT_HOLD"
    GUIDED = "GUIDED"
    GUIDED_NOGPS = "GUIDED_NOGPS"


def change_mode(master: mavutil.mavserial, mode: FlightMode):
    """change flight mode"""
    if not isinstance(master, mavutil.mavserial):
        print("'master' should be instance of 'mavutil.mavserial'.", file=sys.stderr)
        return

    if not isinstance(mode, FlightMode):
        print("'mode' should be a 'FlightMode'.", file=sys.stderr)
        return

    if mode.value not in master.mode_mapping():
        print(f"Unknown mode: {mode.value}", file=sys.stderr)
        print("Try: ", list(master.mode_mapping().keys()), file=sys.stderr)
        return

    ## Get mode ID
    mode_id = master.mode_mapping()[mode.value]
    master.set_mode(mode_id)  # TODO: use long_command_send

    print("Waiting for mode change ACK...")
    # while True:
    #     ack_msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    #     print(ack_msg)

    #     # ack_msg = ack_msg.to_dict()

    #     # Waiting if acknowledged command is not `set_mode`
    #     if ack_msg["command"] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
    #         continue

    #     ## Print ACK result
    #     print(mavutil.mavlink.enums["MAV_RESULT"][ack_msg["result"]].description)
    #     print(mavutil.mavlink.enums["MAV_RESULT"][ack_msg["result"]].name)
    #     break
    ack_msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    print("ack_msg: ", ack_msg)

    print(f"Mode changed to {mode}")


def arm(master):
    """arm the motors"""
    if not isinstance(master, mavutil.mavserial):
        print("'master' should be instance of 'mavutil.mavserial'.", file=sys.stderr)
        return

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    print("Arming motors...")

    # print("Waiting for the vehicle to arm...")
    ack_msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    print("ack_msg: ", ack_msg)

    # ack_msg = ack_msg.to_dict()
    # if ack_msg["result"] == 0:
    #     print("Arming not accepted.")
    #     return 1
    # else:
    #     print("Armed!")


def disarm(master):
    """disarm the motors"""
    if not isinstance(master, mavutil.mavserial):
        print("'master' should be instance of 'mavutil.mavserial'.", file=sys.stderr)
        return

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    print("Disarming motors...")

    # print("Waiting for the vehicle to arm...")
    ack_msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    print("ack_msg: ", ack_msg)


def takeoff(master: mavutil.mavserial, altitude: int):
    """takeoff"""
    if not isinstance(master, mavutil.mavserial):
        print("'master' should be instance of 'mavutil.mavserial'.", file=sys.stderr)
        return

    if not isinstance(altitude, int):
        print("'altitude', should be instance of 'int'.", file=sys.stderr)
        return

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        altitude,
    )

    ack_msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    print("ack_msg: ", ack_msg)


class SpeedType(int, Enum):
    """
    SPEED_TYPE

    https://mavlink.io/en/messages/common.html#SPEED_TYPE
    """

    SPEED_TYPE_AIRSPEED = 0
    SPEED_TYPE_GROUNDSPEED = 1
    SPEED_TYPE_CLIMB_SPEED = 2
    SPEED_TYPE_DESCENT_SPEED = 3


def change_action_speed(
    master: mavutil.mavserial, speed_type: SpeedType, speed: int = -1
):
    """
    Change Action Speed

    https://mavlink.io/en/messages/common.html#MAV_CMD_DO_CHANGE_SPEED

    Arguments:
        - master - ardupilot connection
        - speed_type - one among four speed types
        - speed - in m/s, speed > 0, speed = -1 no change, speed = -2 to return to default speed
    """
    if not isinstance(master, mavutil.mavserial):
        print("'master' should be instance of 'mavutil.mavserial'.", file=sys.stderr)
        return

    if not isinstance(speed_type, SpeedType):
        print("'speed_type' should be instance of 'SpeedType'.", file=sys.stderr)
        return

    if not isinstance(speed, int):
        print("'speed' should be instance of 'int'.", file=sys.stderr)
        return

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0,
        speed_type.value,
        speed,
        -1,
        0,
        0,
        0,
        0,
    )

    ack_msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    print("ack_msg: ", ack_msg)


class RCInputChannel(int, Enum):
    """RC Input Channel for fundamental tasks"""

    ROLL = 1
    PITCH = 2
    THROTTLE = 3
    YAW = 4


def send_rc_pwm(master: mavutil.mavserial, rc_channel: RCInputChannel, pwm: int):
    """
    rend RC PWM input

    https://www.ardusub.com/developers/pymavlink.html#send-rc-joystick

    Arguments:
    - rc_channel - RC Input Channel
    - pwm - pwm input value (1100 - 1800)
    """
    if not isinstance(master, mavutil.mavserial):
        print(
            "master parameter should be instance of mavutil.mavserial.", file=sys.stderr
        )
        return

    if not isinstance(rc_channel, RCInputChannel):
        print(
            "rc_channel parameter should be instance of RCInputChannel.",
            file=sys.stderr,
        )
        return

    if not isinstance(pwm, int):
        print("pwm parameter should be instance of int.", file=sys.stderr)
        return

    if pwm < 1100 or pwm > 1800:
        print("pwm should be in the range 1100 to 1800.", file=sys.stderr)
        return

    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[rc_channel.value - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component, *rc_channel_values
    )

    # ack_msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    # print("ack_msg: ", ack_msg)


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

    menu = [
        "View ardupilot status (not implemented)",
        "Change mode",
        "Arm",
        "Disarm",
        "Set input throttle PWM",
        "Takeoff"
    ]

    while True:
        print()
        print("MENU:")
        for i, menu_item in enumerate(menu):
            print(f"{i + 1}. {menu_item}")
        print("(Q/q)uit")
        print(f"Enter your choice (1 - {len(menu) + 1} or q/Q to quit): ", end="")

        choice = input()

        if choice == "q" or choice == "Q":
            break

        try:
            choice_int = int(choice)
        except ValueError:
            print("Enter a number for choice")
            continue

        if choice_int < 1 or choice_int > len(menu):
            print("Wrong choice. Try again.", file=sys.stderr)
            continue

        if choice_int == 1:
            print("(Not implemented)")
        elif choice_int == 2:
            print("Mode choice:")
            flight_modes = [mode.value for mode in FlightMode]
            for i, mode in enumerate(flight_modes):
                print(f"{i + 1} {mode}")

            print("Enter flight mode choice: ", end="")
            try:
                flight_mode_choice = int(input())
            except ValueError:
                print("Choice should be an integer.", file=sys.stderr)
                continue

            if choice < 1 or choice > len(flight_modes):
                print("Wrong flight mode choice.", file=sys.stderr)
                continue

            flight_mode = FlightMode[flight_modes[flight_mode_choice - 1]]

            change_mode(master, flight_mode)
        elif choice_int == 3:
            arm(master)
        elif choice_int == 4:
            disarm(master)
        elif choice_int == 5:
            try:
                pwm_value = int(input("Enter PWM value: "))
            except ValueError:
                print("Enter an integer value for PWM.", file=sys.stderr)
                continue

            send_rc_pwm(master, RCInputChannel.THROTTLE, pwm_value)
        elif choice_int == 6:
            try:
                altitude_value = int(input("Enter takeoff altitude: "))
            except ValueError:
                print("Enter an integer value for takeoff altitudde.", file=sys.stderr)
                continue

            takeoff(master, altitude_value)

    master.close()
    print("Connection closed.")

    return 0


if __name__ == "__main__":
    sys.exit(main(args=sys.argv))
