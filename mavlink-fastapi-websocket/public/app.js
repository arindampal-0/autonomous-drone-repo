console.log("FastAPI WebSocket server")

/** @typedef { "STATE" | "CONNECT" | "MODE_CHANGE" | "ARM" | "DISARM" | "RUN_MOTOR" | "STOP_MOTOR" | "CLOSE_CONNECTION" } WebSocketSendMsgType */

/** @typedef {{ msg_type: "STATE" } | { msg_type: "CONNECT", device_connection_string: string } | { msg_type: "MODE_CHANGE", mode: FlightMode } | { msg_type: "ARM" } | { msg_type: "DISARM" } | { msg_type: "RUN_MOTOR", speed: number } | { msg_type: "STOP_MOTOR" } | { msg_type: "CLOSE_CONNECTION" }} WebSocketSendMsg */

const ws = new WebSocket("/ws")

const refreshStateButton = document.getElementById("refresh-button");
// console.log(refreshStateButton);

const deviceConnectionForm = document.getElementById("device-connection-string-form");
// console.log(deviceConnectionForm);

const deviceConnectionSubmitButton = document.querySelector("#device-connection-string-form button[type='submit']");
// console.log(deviceConnectionSubmitButton);

const deviceConnectionStrInput = document.getElementById("device-connection-string-input");
// console.log(deviceConnectionStrInput)

const connectionStateDiv = document.getElementById("connection-status");
// console.log(connectionStateDiv);

const modeChangeForm = document.getElementById("mode-change-form");
// console.log(modeChangeForm);

const modeSelect = document.getElementById("select-mode");
// console.log(modeSelect);

const currentModeSpan = document.getElementById("current-mode");
// console.log(currentModeSpan);

const armButton = document.getElementById("arm-button");
// console.log(armButton);

const armStatusSpan = document.getElementById("arm-status");
// console.log(armStatusSpan);

const runMotorForm = document.getElementById("run-motor-form");
// console.log(runMotorForm);

const speedRangeInput = document.getElementById("motor-speed");
// console.log(speedRangeInput);

const speedNumberInput = document.getElementById("motor-speed-value");
// console.log(speedNumberInput);

const motorSpinningStatusSpan = document.getElementById("motor-status");
// console.log(motorSpinningStatusSpan);

/** @typedef {"STABILIZE" | "NONE"} FlightMode */

/** @type { {connected: bool, mode: FlightMode, armed: bool, motorSpinning: bool, motorSpeed: number }} */
const ardupilotState = {
    connected: false,
    mode: "NONE",
    armed: false,
    motorSpinning: false,
    motorSpeed: 0
};

function updateUIState() {
    // update the device connection UI
    if (deviceConnectionSubmitButton instanceof HTMLButtonElement) {
        deviceConnectionSubmitButton.innerText = ardupilotState.connected ? "Disconnect": "Connect";
    } else {
        console.error("Could not find submit button for #device-connection-string-form.");
    }

    if (deviceConnectionStrInput instanceof HTMLInputElement) {
        deviceConnectionStrInput.disabled = ardupilotState.connected;
    } else {
        console.error("Could not find input field #device-connection-string-input.");
    }

    if (connectionStateDiv instanceof HTMLDivElement) {
        connectionStateDiv.innerText = ardupilotState.connected ? "CONNECTED" : "Not Connected";
        connectionStateDiv.style.color = ardupilotState.connected ? "green" : "red";
    } else {
        console.error("Could not find div #connection-state.")
    }
}

/**
 * send connection message
 * @param {bool} connect Want to connect or disconnect
 * @param {string} deviceConnectionString device connection string
 */
function sendConnectionMessage(connect, deviceConnectionString = "") {
    if (!(connect instanceof bool)) {
        console.error("'connect' parameter should be a boolean.");
        return;
    }

    if (!(deviceConnectionString instanceof String)) {
        console.error("'deviceConnectionString' should be a String.");
        return;
    }

    const message = { "msg_type": "" };
    if (connect) {
        if (deviceConnectionString.length == 0) {
            console.error("'deviceConnectionString' should be non-empty.");
            return;
        }

        message["msg_type"] = "CONNECT";
        message["device_connection_string"] = deviceConnectionString;
    } else {
        message["msg_type"] = "CLOSE_CONNECTION";
    }

    if (ws.OPEN) {
        ws.send(JSON.stringify(message));
    } else {
        console.error("WS connection is closed.");
    }
}


if (refreshStateButton instanceof HTMLButtonElement) {
    refreshStateButton.addEventListener("click", function() {
        /** @type {WebSocketSendMsg} */
        const message = { msg_type: "STATE" }

        if (ws.OPEN) {
            ws.send(JSON.stringify({msg_type}))
        }
    });
} else {
    console.error("Cound not get #refresh-button from dom.");
}

if (deviceConnectionForm instanceof HTMLFormElement) {
    deviceConnectionForm.addEventListener("submit", function(event) {
        event.preventDefault();

        if (ardupilotState.connected) {
            if (deviceConnectionStrInput instanceof HTMLInputElement) {
                const deviceConnectionString = deviceConnectionStrInput.value;
                sendConnectionMessage(true, deviceConnectionString);
            }
        } else {
            sendConnectionMessage(false);
        }
    });
} else {
    console.error("Could not get #device-connection-string-form from dom.")
}

ws.addEventListener("open", function(event) {
    console.log("Websocket connection opened!");
});

ws.addEventListener("message", function(event) {
    console.log("message: ", event.data);
    const message = JSON.parse(event.data);

    if (!message.hasOwnProperty("msg_type")) {
        console.error("message should have msg_type property.");
        return;
    }

    if (message.msg_type != "STATE") {
        console.error("Only msg_type = 'STATE' is allowed.");
        return;
    }

    if (!message.hasOwnProperty("state")) {
        console.error("STATE message type should have 'state' property.")
        return;
    }

    if (!message.state.hasOwnProperty("connected")) {
        console.error("message.state should have 'connected' property.");
        return;
    }

    if (!(message.state.connected instanceof bool)) {
        console.error("message.state.connected should be a boolean value.");
        return;
    }

    if (!message.state.hasOwnProperty("mode")) {
        console.error("message.state should have 'mode' property.");
        return;
    }

    if ( !(message.state.mode == "STATE" || 
        message.state.mode == "CONNECT" ||
        message.state.mode == "MODE_CHANGE" ||
        message.state.mode == "ARM" ||
        message.state.mode == "DISARM" ||
        message.state.mode == "RUN_MOTOR" ||
        message.state.mode == "STOP_MOTOR" ||
        message.state.mode == "CLOSE_CONNECTION")) {
            console.error("message.state.mode is invalid!");
            return;
        }
    
    if (!message.state.hasOwnProperty("armed")) {
        console.error("message.state should have 'arm' property.");
        return;
    }

    if (!(message.state.armed instanceof bool)) {
        console.error("message.state.armed should be a boolean value.");
        return;
    }

    if (!message.state.hasOwnProperty("motor_spinning")) {
        console.error("message.state should have 'motor_spinning' property.");
        return;
    }

    if (!(message.state.motor_spinning instanceof bool)) {
        console.error("message.state.motor_spinning should be a boolean value.");
        return;
    }

    if (!message.state.hasOwnProperty("motor_speed")) {
        console.error("message.state should have 'motor_spped' property.");
        return;
    }

    if (!(message.state.motor_speed instanceof number)) {
        console.error("message.state.motor_speed should be a boolean value.");
        return;
    }

    if (message.state.speed < 1100 && message.state.speed > 1900) {
        console.error("message.state.motor_speed should be between 1100 and 1900.");
        return;
    }

    ardupilotState.connected = message.state.connected;
    ardupilotState.mode = message.state.mode;
    ardupilotState.armed = message.state.armed;
    ardupilotState.motorSpinning = message.state.motor_spinning;
    ardupilotState.motorSpeed = message.state.motor_speed;

    updateUIState();
});

ws.addEventListener("error", function(event) {
    console.error("Websocket errored: ", event.error);
});

ws.addEventListener("close", function(event) {
    console.log("Websocket connection closed!");
});

