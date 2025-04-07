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


class WebSocketRecvMsgType(str, Enum):
    """Enum representing WebSocket JSON Receive Message Types"""

    STATE = "STATE"
    CONNECT = "CONNECT"
    MODE_CHANGE = "MODE_CHANGE"
    ARM = "ARM"
    DISARM = "DISARM"
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

async def connect(device_connection_string: str):
    """connect to pixhawk"""
    global master
    # connect to pixhawk
    master = mavutil.mavserial(device_connection_string, baud=115200)

    if pixhawk_is_connected():
        await print_all_flight_modes()
        ardupilot_state["connected"] = True
    else:
        print("CONNECTION failed!", file=sys.stderr)


async def print_all_flight_modes():
    """print all the available flight modes"""
    global master

    if not master:
        print("Ardupilot not connected.", file=sys.stderr)
        return

    print("Flight Modes: ", list(master.mode_mapping().keys()))


async def change_flight_mode(mode: FlightMode):
    """change flight mode"""
    global master

    if not master:
        print("Ardupilot not connected.", file=sys.stderr)
        return

    if mode.name not in master.mode_mapping():
        print(f"Unknown mode: {mode.name}", file=sys.stderr)
        print("Try: ", list(master.mode_mapping().keys()), file=sys.stderr)
        return

    ## Get mode ID
    mode_id = master.mode_mapping()[mode.name]
    master.set_mode(mode_id)

    print("Waiting for mode change ACK...")
    while True:
        ack_msg = master.recv_match(type="COMMAND_ACK", blocking=True)
        print(ack_msg.message_type)
        print(ack_msg.get_type())
        print(ack_msg.get_fieldnames())

        ack_msg = ack_msg.to_dict()

        ## Waiting if acknowledged command is not `set_mode`
        if ack_msg["command"] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            continue

        ## Print ACK result
        print(mavutil.mavlink.enums["MAV_RESULT"][ack_msg["result"]].description)
        break
    print(f"Mode changed to {mode.name}")

    ardupilot_state["mode"] = mode


async def arm_copter():
    """arming the copter"""
    global master

    if not master:
        print("Ardupilot not connected.", file=sys.stderr)
        return

    master.arducopter_arm()
    print("Arming motors.")

    print("Waiting for the vehicle to arm...")
    master.motors_armed_wait()
    print("Armed!")

    ardupilot_state["armed"] = True


async def disarm_copter():
    """disarming the copter"""
    global master

    if not master:
        print("Ardupilot not connected.", file=sys.stderr)
        return

    master.arducopter_disarm()
    print("Disarming motors.")

    print("Waiting for the vehicle to disarm...")
    master.motors_disarmed_wait()
    print("Motors disarmed!")

    ardupilot_state["armed"] = False


async def run_motors(speed: int):
    """running motors"""
    global master

    if not master:
        print("Ardupilot not connected.", file=sys.stderr)
        return

    if not isinstance(speed, int):
        print("speed should be integer.", file=sys.stderr)
        return

    if speed < 1100 and speed > 1900:
        print("speed should be between 1100 and 1900, both included.", file=sys.stderr)
        return

    master.set_servo(1 + 8, speed)

    ardupilot_state["motor_spinning"] = True
    ardupilot_state["motor_speed"] = speed


async def stop_motors():
    """stop motors from spinning"""
    global master

    if not master:
        print("Ardupilot not connected.", file=sys.stderr)
        return

    master.set_servo(1 + 8, 0)

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

        await connect(message["device_connection_string"])
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

        await change_flight_mode(mode)
    elif msg_type == WebSocketRecvMsgType.ARM:
        await arm_copter()
    elif msg_type == WebSocketRecvMsgType.DISARM:
        await disarm_copter()
    elif msg_type == WebSocketRecvMsgType.RUN_MOTOR:
        if "speed" not in message:
            print(
                "speed property should be part of RUN_MOTOR message type.",
                file=sys.stderr,
            )
            return

        if not isinstance(message["speed"], int):
            print("speed property should be an integer.", file=sys.stderr)
            return

        await run_motors(message["speed"])
    elif msg_type == WebSocketRecvMsgType.STOP_MOTOR:
        await stop_motors()
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
