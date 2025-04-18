"""main file"""

from typing import Union

import sys
from enum import Enum
import time

import uvicorn
from fastapi import FastAPI, WebSocket
from fastapi.staticfiles import StaticFiles
from pymavlink import mavutil

master: Union[mavutil.mavserial, None] = None


class WebSocketSendMsgType(str, Enum):
    """Enum representing WebSocket JSON Send Message Types"""

    STATE = "STATE"
    LOG_MESSAGE = "LOG_MESSAGE"


class WebSocketRecvMsgType(str, Enum):
    """Enum representing WebSocket JSON Receive Message Types"""

    STATE = "STATE"
    CONNECT = "CONNECT"
    MODE_CHANGE = "MODE_CHANGE"
    ARM = "ARM"
    DISARM = "DISARM"
    MOTOR_TEST = "MOTOR_TEST"
    RUN_MOTOR = "RUN_MOTOR"
    STOP_MOTOR = "STOP_MOTOR"
    CLOSE_CONNECTION = "CLOSE_CONNECTION"

    @staticmethod
    def from_str(msg_type: str) -> Union["WebSocketRecvMsgType", None]:
        """get recieve message type"""
        for t in WebSocketRecvMsgType:
            if t.name == msg_type:
                return t

        return None


class FlightMode(str, Enum):
    """Enum representing flight modes"""

    STABILIZE = "STABILIZE"
    ALT_HOLD = "ALT_HOLD"
    LAND = "LAND"
    NONE = "NONE"

    @staticmethod
    def from_str(mode: str) -> Union["FlightMode", None]:
        """get flight mode"""
        for m in FlightMode:
            if m.name == mode:
                return m
        return None


ardupilot_state = {
    "connected": False,
    "device_connection_string": "",
    "mode": FlightMode.NONE,
    "armed": False,
    "motor_spinning": False,
    "motor_speed": 0,  # pwm
}


async def send_state(ws: WebSocket):
    """send ardupilot state"""
    if not isinstance(ws, WebSocket):
        print("ws should be instance of WebSocket.", file=sys.stderr)
        return

    await ws.send_json(
        {"msg_type": WebSocketSendMsgType.STATE, "state": ardupilot_state}
    )

async def send_log_message(ws: WebSocket, message):
    """send log message"""
    if not isinstance(ws, WebSocket):
        print("ws should be instance of WebSocket.", file=sys.stderr)
        return

    await ws.send_json(
        {"msg_type": "LOG_MESSAGE", "message": message}
    )

def pixhawk_is_connected() -> bool:
    """check if ardupilot is connected"""
    global master

    if not master:
        print("Ardupilot not connected.", file=sys.stderr)
        return False

    start_time = time.time()

    # make sure connection is valid
    master.wait_heartbeat(blocking=True, timeout=10)

    return time.time() - start_time < 8

async def connect(ws: WebSocket, device_connection_string: str):
    """connect to pixhawk"""
    global master
    # connect to pixhawk
    master = mavutil.mavserial(device_connection_string, baud=115200)

    if pixhawk_is_connected():
        await print_all_flight_modes(ws)
        ardupilot_state["connected"] = True
        ardupilot_state["device_connection_string"] = device_connection_string
        await send_log_message(ws, f"{device_connection_string} CONNECTED!")
    else:
        print("CONNECTION failed!", file=sys.stderr)
        await send_log_message(ws, f"{device_connection_string} CONNECTION failed!")


async def print_all_flight_modes(ws: WebSocket):
    """print all the available flight modes"""
    global master

    if not master:
        print("Ardupilot not connected.", file=sys.stderr)
        return

    print("Flight Modes: ", list(master.mode_mapping().keys()))
    await send_log_message(ws, str(list(master.mode_mapping().keys())))


async def change_flight_mode(ws: WebSocket, mode: FlightMode):
    """change flight mode"""
    global master

    if not master:
        print("Ardupilot not connected.", file=sys.stderr)
        return

    if mode.name not in master.mode_mapping():
        print(f"Unknown mode: {mode.name}", file=sys.stderr)
        await send_log_message(ws, f"Unknown mode: {mode.name}")
        print("Try: ", list(master.mode_mapping().keys()), file=sys.stderr)
        return

    ## Get mode ID
    mode_id = master.mode_mapping()[mode.name]
    master.set_mode(mode_id)

    print("Waiting for mode change ACK...")
    arm_success = False
    for _ in range(5):
        ack_msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=2)
        if not ack_msg:
            continue

        ack_msg = ack_msg.to_dict()

        ## Waiting if acknowledged command is not `set_mode`
        if ack_msg["command"] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            continue

        ## Print ACK result
        print(mavutil.mavlink.enums["MAV_RESULT"][ack_msg["result"]].description)
        print(f"Mode changed to {mode.name}")
        await send_log_message(ws, f"Mode changed to {mode.name}")
        ardupilot_state["mode"] = mode
        arm_success = True
        break

    if not arm_success:
        print("Failed to change flight mode.", file=sys.stderr)
        await send_log_message(ws, "Failed to change flight mode.")


async def arm_copter(ws: WebSocket):
    """arming the copter"""
    global master

    if not master:
        print("Ardupilot not connected.", file=sys.stderr)
        return

    master.arducopter_arm()
    print("Arming motors.")

    print("Waiting for the vehicle to arm...")
    ack_msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    print("ack_msg: ", ack_msg)

    if not ack_msg:
        print("Arming failed!", file=sys.stderr)
        await send_log_message(ws, "Arming failed!")
    else:
        print("Armed!")
        await send_log_message(ws, "Motor armed.")
        ardupilot_state["armed"] = True


async def disarm_copter(ws: WebSocket):
    """disarming the copter"""
    global master

    if not master:
        print("Ardupilot not connected.", file=sys.stderr)
        return

    master.arducopter_disarm()
    print("Disarming motors.")

    print("Waiting for the vehicle to disarm...")
    ack_msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    print("ack_msg: ", ack_msg)

    if not ack_msg:
        print("Disarming failed!", file=sys.stderr)
        await send_log_message(ws, "Disarming failed!")
    else:
        print("Motors disarmed!")
        await send_log_message(ws, "Motors disarmed.")
        ardupilot_state["armed"] = False


async def run_motor_tests(ws: WebSocket, throttle_percentage: int = 5):
    """run motor tests"""
    global master

    if not master:
        print("Ardupilot not connected.", file=sys.stderr)
        return

    if not isinstance(throttle_percentage, int):
        print("throttle_percentage should be integer.", file=sys.stderr)
        return

    if throttle_percentage < 0 or throttle_percentage > 100:
        print("throttle_percentage argument should be between 1 and 100")

    master.mav.command_long_send(master.target_system, master.target_component,
                                mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
                                1, # Instance
                                0, # throttle type: MOTOR_TEST_THROTTLE_PERCENT
                                throttle_percentage, # throttle value
                                5, # timeout between tests, min:0
                                4, # motor count, number of motors to test in sequence
                                0, # motor test order: MOTOR_TEST_ORDER_DEFAULT
                                0) # EMPTY
    ack_msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    print("ack_msg: ", str(ack_msg))
    await send_log_message(ws, str(ack_msg))


async def run_motors(speed: int, channel: int):
    """running motors"""
    global master

    if not master:
        print("Ardupilot not connected.", file=sys.stderr)
        return

    if not isinstance(speed, int):
        print("speed should be integer.", file=sys.stderr)
        return

    if not isinstance(channel, int):
        print("channel should be integer.", file=sys.stderr)
        return

    if speed < 1100 and speed > 1900:
        print("speed should be between 1100 and 1900, both included.", file=sys.stderr)
        return

    # master.set_servo(1 + 8, speed)
    master.set_servo(channel, speed)

    ardupilot_state["motor_spinning"] = True
    ardupilot_state["motor_speed"] = speed


async def stop_motors(channel: int):
    """stop motors from spinning"""
    global master

    if not master:
        print("Ardupilot not connected.", file=sys.stderr)
        return

    if not isinstance(channel, int):
        print("channel should be integer.", file=sys.stderr)
        return

    master.set_servo(channel, 0)

    ardupilot_state["motor_spinning"] = False
    ardupilot_state["motor_speed"] = 0

async def close_connection():
    """close connection to ardupilot"""
    global master

    if not master:
        print("Ardupilot not connection.", file=sys.stderr)
        return

    master.close()
    master = None
    ardupilot_state["connected"] = False
    ardupilot_state["device_connection_string"] = ""
    ardupilot_state["mode"] = FlightMode.NONE
    ardupilot_state["armed"] = False
    ardupilot_state["motor_spinning"] = False
    ardupilot_state["motor_speed"] = 0


app = FastAPI(title="app")

api = FastAPI(title="api")


@api.get("/health")
def health_route():
    """health route handler"""
    return {"status": "OK"}


async def handle_websocket_json_messages(message: dict, ws: WebSocket):
    """handle WebSocket JSON messages"""
    print(message)
    if not isinstance(message, dict):
        print("message should be a dictiionary.", file=sys.stderr)
        return

    if "msg_type" not in message:
        print("msg_type should be part of json message.", file=sys.stderr)
        return

    msg_type = WebSocketRecvMsgType.from_str(message["msg_type"])
    if not msg_type:
        print(f"msg_type '{message['msg_type']}' is invalid.", file=sys.stderr)
        return

    if msg_type == WebSocketRecvMsgType.STATE:
        await send_state(ws)
    elif msg_type == WebSocketRecvMsgType.CONNECT:
        if "device_connection_string" not in message:
            print(
                "device_connection_string should be part of CONNECT message type.",
                file=sys.stderr,
            )
            return
        if not isinstance(message["device_connection_string"], str):
            print(
                "device_connection_string property should be a string.",
                file=sys.stderr,
            )
            return

        await connect(ws, message["device_connection_string"])
    elif msg_type == WebSocketRecvMsgType.MODE_CHANGE:
        if "mode" not in message:
            print(
                "mode property should be part of MODE_CHANGE message type.",
                file=sys.stderr,
            )
            return

        if not isinstance(message["mode"], str):
            print("mode property should be a string.", file=sys.stderr)
            return

        mode = FlightMode.from_str(message["mode"])

        if not mode:
            print(f"'{message['mode']} is an invalid flight mode.", file=sys.stderr)
            return

        await change_flight_mode(ws, mode)
    elif msg_type == WebSocketRecvMsgType.ARM:
        await arm_copter(ws)
    elif msg_type == WebSocketRecvMsgType.DISARM:
        await disarm_copter(ws)
    elif msg_type == WebSocketRecvMsgType.MOTOR_TEST:
        if "throttle_percentage" not in message:
            print(
                "throttle_percentage property should be part of MOTOR_TEST message type",
                file=sys.stderr
            )
            return

        if not isinstance(message["throttle_percentage"], int):
            print(
                "throttle_percentage property should be an integer.",
                file=sys.stderr
            )
            return

        await run_motor_tests(ws, message["throttle_percentage"])

    elif msg_type == WebSocketRecvMsgType.RUN_MOTOR:
        if "speed" not in message:
            print(
                "speed property should be part of RUN_MOTOR message type.",
                file=sys.stderr,
            )
            return

        if "channel" not in message:
            print(
                "channel property should be part of RUN_MOTOR message type.",
                file=sys.stderr
            )
            return

        if not isinstance(message["speed"], int):
            print("speed property should be an integer.", file=sys.stderr)
            return

        if not isinstance(message["channel"], int):
            print("channel property should be an integer.", file=sys.stderr)
            return

        await run_motors(message["speed"], message["channel"])
    elif msg_type == WebSocketRecvMsgType.STOP_MOTOR:
        if "channel" not in message:
            print(
                "channel property should be part of RUN_MOTOR message type.",
                file=sys.stderr
            )
            return

        if not isinstance(message["channel"], int):
            print("channel property should be an integer.", file=sys.stderr)
            return

        await stop_motors(message["channel"])
    elif msg_type == WebSocketRecvMsgType.CLOSE_CONNECTION:
        await close_connection()


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """websocket handler"""
    await websocket.accept()
    async for message in websocket.iter_json():
        await handle_websocket_json_messages(message, websocket)
        await send_state(websocket)


app.mount("/api", api, name="api")

app.mount("/", StaticFiles(directory="public", html=True), name="static")


def main():
    """main function"""
    uvicorn.run(app, host="0.0.0.0", port=8000)


if __name__ == "__main__":
    main()
