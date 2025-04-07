console.log("FastAPI WebSocket server")

/** @typedef { "STATE" | "CONNECT" | "MODE_CHANGE" | "ARM" | "DISARM" | "RUN_MOTOR" | "STOP_MOTOR" | "CLOSE_CONNECTION" } WebSocketSendMsgType */

/** @typedef {{ msg_type: "STATE" } | { msg_type: "CONNECT", device_connection_string: string } | { msg_type: "MODE_CHANGE", mode: FlightMode } | { msg_type: "ARM" } | { msg_type: "DISARM" } | { msg_type: "RUN_MOTOR", speed: number, channel: number } | { msg_type: "STOP_MOTOR", channel: number } | { msg_type: "CLOSE_CONNECTION" }} WebSocketSendMsg */

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

const disarmButton = document.getElementById("disarm-button");
// console.log(disarmButton);

const armStatusSpan = document.getElementById("arm-status");
// console.log(armStatusSpan);

const runMotorForm = document.getElementById("run-motor-form");
// console.log(runMotorForm);

const runMotorButton = document.querySelector("#run-motor-form button[type='submit']");
// console.log(runMotorButton);

const stopMotorButton = document.getElementById("stop-motor-button");
// console.log(stopMotorButton);

const speedRangeInput = document.getElementById("motor-speed");
// console.log(speedRangeInput);

const speedNumberInput = document.getElementById("motor-speed-value");
// console.log(speedNumberInput);

const servoChannelInput = document.getElementById("servo-channel");
// console.log(servoChannelInput);

const motorSpinningStatusSpan = document.getElementById("motor-status");
// console.log(motorSpinningStatusSpan);

/** @typedef {"STABILIZE" | "NONE"} FlightMode */

/** @type { {connected: boolean, deviceConnectionString: string, mode: FlightMode, armed: boolean, motorSpinning: boolean, motorSpeed: number }} */
const ardupilotState = {
    connected: false,
    deviceConnectionString: "",
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
        deviceConnectionStrInput.value = ardupilotState.deviceConnectionString;
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

    // update mode change UI
    if (currentModeSpan instanceof HTMLSpanElement) {
        if (ardupilotState.mode == "NONE") {
            currentModeSpan.innerText = "Unknown";
            currentModeSpan.style.color = "red";
        } else {
            currentModeSpan.innerText = ardupilotState.mode;
            currentModeSpan.style.color = "white";
        }
    } else {
        console.error("Could not find #current-mode.");
    }

    if (armStatusSpan instanceof HTMLSpanElement) {
        armStatusSpan.innerText = ardupilotState.armed ? "Armed" : "Disarmed";
        armStatusSpan.style.color = ardupilotState.armed ? "red" : "white";
    } else {
        console.error("Could not find #arm-status.");
    }

    // update run motor UI
    if (runMotorButton instanceof HTMLButtonElement) {
        // runMotorButton.disabled = ardupilotState.motorSpinning;
    } else {
        console.error("Could not find run-motor-button in dom");
    }

    if (stopMotorButton instanceof HTMLButtonElement) {
        // stopMotorButton.disabled = !ardupilotState.motorSpinning;
    } else {
        console.error("Could not find #stop-motor-button in dom");
    }

    if (speedRangeInput instanceof HTMLInputElement) {
        speedRangeInput.value = ardupilotState.motorSpeed;
    } else {
        console.error("Could not find speed-range-input in dom.");
    }

    if (speedNumberInput instanceof HTMLInputElement) {
        speedNumberInput.value = ardupilotState.motorSpeed;
    } else {
        console.error("Could not find speed-number-input in dom.");
    }

    if (motorSpinningStatusSpan instanceof HTMLSpanElement) {
        motorSpinningStatusSpan.innerText = ardupilotState.motorSpinning ? "Spinning" : "Not spinning";
        motorSpinningStatusSpan.style.color = ardupilotState.motorSpinning ? "green" : "red";
    } else {
        console.error("Could not find motor-spinning-status in dom.");
    }
}
 
updateUIState();

/**
 * send state message
 */
function sendStateMessage() {
    if (!(ws instanceof WebSocket)) {
        console.error("ws should be instance of WebSocket");
        return;
    }

    /** @type {WebSocketSendMsg} */
    const message = { msg_type: "STATE" }

    if (ws.OPEN) {
        console.log(message)
        ws.send(JSON.stringify(message));
    } else {
        console.error("WS connection is closed.");
    }
}

/**
 * send connection message
 * @param {boolean} connect Want to connect or disconnect
 * @param {string} deviceConnectionString device connection string
 */
function sendConnectionMessage(connect, deviceConnectionString = "") {
    if (typeof connect != "boolean") {
        console.error("'connect' parameter should be a boolean.");
        return;
    }

    if (typeof deviceConnectionString != "string") {
        console.error("'deviceConnectionString' should be a string.");
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
        console.log(message)
        ws.send(JSON.stringify(message));
    } else {
        console.error("WS connection is closed.");
    }
}

/**
 * Send mode change message
 * @param {FlightMode} mode flight mode
 */
function sendModeChangeMessage(mode) {
    if (typeof mode != "string") {
        console.error("'mode' parameter should be a string.");
        return;
    }

    if (ws.OPEN) {
        ws.send(JSON.stringify({ msg_type: "MODE_CHANGE", mode }));
    } else {
        console.error("WS connection is closed.");
    }
}

/**
 * Send message to arm or disarm the motor
 * @param {boolean} arm arm the motor (true), disarm the motor (false)
 */
function sendArmMessage(arm) {
    if (typeof arm != "boolean") {
        console.error("'arm' parameter should be a boolean.");
        return;
    }

    if (ws.OPEN) {
        const message = { msg_type: arm ? "ARM" : "DISARM" }
        ws.send(JSON.stringify(message));
    } else {
        console.error("WS connection is closed.");
    }
}

/**
 * Send message to spin motor with a speed
 * @param {number} speed speed of motor (1100 <= speed <= 1800)
 * @param {number} channel servo channel
 */
function sendSpinMotorMessage(speed, channel) {
    if (typeof speed != "number") {
        console.error("'speed' parameter should be a number.");
        return;
    }

    if (typeof channel != "number") {
        console.error("'channel' parameter should be a number.")
        return;
    }

    if (speed < 1100 && speed > 1800) {
        console.error("'speed' should be between 1100 and 1800.");
        return;
    }

    if (ws.OPEN) {
        ws.send(JSON.stringify({ msg_type: "RUN_MOTOR", speed, channel }));
    } else {
        console.error("WS connection is closed.");
    }
}

/**
 * Send message to stop the motor
 * @param {number} channel servo channel
 */
function sendStopMotorMessage(channel) {
    if (ws.OPEN) {
        ws.send(JSON.stringify({ msg_type: "STOP_MOTOR", channel }));
    } else {
        console.error("WS connection is closed.");
    }
}


if (refreshStateButton instanceof HTMLButtonElement) {
    refreshStateButton.addEventListener("click", function() {
        sendStateMessage();
    });
} else {
    console.error("Cound not get #refresh-button from dom.");
}

if (deviceConnectionForm instanceof HTMLFormElement) {
    deviceConnectionForm.addEventListener("submit", function(event) {
        event.preventDefault();

        if (!ardupilotState.connected) {
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

if (modeChangeForm instanceof HTMLFormElement) {
    modeChangeForm.addEventListener("submit", function(event) {
        event.preventDefault();

        if (!ardupilotState.connected) {
            console.error("Ardupilot not connected!");
            return;
        }

        if (modeSelect instanceof HTMLSelectElement) {
            const mode = modeSelect.value;
            sendModeChangeMessage(mode);
            modeChangeForm.reset();
        } else {
            console.error("Could not get #mode-select from dom.");
            return;
        }
    });
} else {
    console.error("Could not get #mode-change-form from dom.");
}

if (armButton instanceof HTMLButtonElement) {
    armButton.addEventListener("click", function() {
        if (!ardupilotState.connected) {
            console.error("Ardupilot not connected!");
            return;
        }

        if (ardupilotState.mode === "NONE") {
            console.error("Flight mode is not set.");
            return;
        }

        sendArmMessage(true);
    });
} else {
    console.error("Could not find #arm-button in the dom.");
}

if (disarmButton instanceof HTMLButtonElement) {
    disarmButton.addEventListener("click", function() {
        if (!ardupilotState.connected) {
            console.error("Ardupilot not connected!");
            return;
        }

        if (ardupilotState.mode === "NONE") {
            console.error("Flight mode is not set.");
            return;
        }

        sendArmMessage(false);
    });
} else {
    console.error("Could not find #disarm-button in the dom.");
}

if (speedRangeInput instanceof HTMLInputElement) {
    speedRangeInput.addEventListener("change", function(event) {
        if (speedNumberInput instanceof HTMLInputElement) {
            speedNumberInput.value = event.target.value;
        } else {
            console.error("Could not find #speed-number-input in the dom.");
        }
    });
} else {
    console.error("Could not find #speed-range-input in the dom.");
}

if(speedNumberInput instanceof HTMLInputElement) {
    speedNumberInput.addEventListener("change", function(event) {
        if (speedRangeInput instanceof HTMLInputElement) {
            speedRangeInput.value = event.target.value;
        } else {
            console.error("Could not find #speed-range-input in the dom.");
        }
    });
} else {
    console.error("Could not find #speed-number-input in the dom.");
}

if (runMotorForm instanceof HTMLFormElement) {
    runMotorForm.addEventListener("submit", function(event) {
        event.preventDefault();

        if (!ardupilotState.connected) {
            console.error("Ardupilot not connected!");
            return;
        }

        if (ardupilotState.mode === "None") {
            console.error("Flight mode is not set.");
            return;
        }

        if (!ardupilotState.armed) {
            console.error("motors are not armed.");
            return;
        }

        if (speedNumberInput instanceof HTMLInputElement && servoChannelInput instanceof HTMLInputElement) {
            const speed = parseInt(speedNumberInput.value, 10);
            const channel = parseInt(servoChannelInput.value, 10);
            sendSpinMotorMessage(speed, channel);
        } else {
            console.error("Could not find #speed-number-input or #servo-channel in the dom.");
        }
    })
} else {
    console.error("Could not find #run-motor-form in the dom.");
}

if (stopMotorButton instanceof HTMLButtonElement) {
    stopMotorButton.addEventListener("click", function() {
        if (!ardupilotState.connected) {
            console.error("Ardupilot not connected!");
            return;
        }

        if (ardupilotState.mode === "None") {
            console.error("Flight mode is not set.");
            return;
        }

        if (!ardupilotState.armed) {
            console.error("motors are not armed.");
            return;
        }

        if (!ardupilotState.motorSpinning) {
            console.log("Motor is not spinning!");
            return;
        }

        if (servoChannelInput instanceof HTMLInputElement) {
            const channel = parseInt(servoChannelInput.value, 10);
            sendStopMotorMessage(channel);
        }
    });
} else {
    console.error("Could not find #stop-motor-button in the dom.");
}

ws.addEventListener("open", function(event) {
    console.log("Websocket connection opened!");
    sendStateMessage();
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

    if (typeof message.state.connected !== "boolean") {
        console.error("message.state.connected should be a boolean value.");
        return;
    }

    if (!message.state.hasOwnProperty("device_connection_string")) {
        console.error("message.state should have 'device_connection_string' property.");
        return;
    }

    if (typeof message.state.device_connection_string !== "string") {
        console.error("message.state.device_connection_string should be a string value.");
        return;
    }

    if (!message.state.hasOwnProperty("mode")) {
        console.error("message.state should have 'mode' property.");
        return;
    }

    if ( !(message.state.mode === "STABILIZE" || 
        message.state.mode === "ALT_HOLD" ||
        message.state.mode === "NONE")) {
            console.error("message.state.mode is invalid!");
            return;
        }
    
    if (!message.state.hasOwnProperty("armed")) {
        console.error("message.state should have 'arm' property.");
        return;
    }

    if (typeof message.state.armed !== "boolean") {
        console.error("message.state.armed should be a boolean value.");
        return;
    }

    if (!message.state.hasOwnProperty("motor_spinning")) {
        console.error("message.state should have 'motor_spinning' property.");
        return;
    }

    if (typeof message.state.motor_spinning !== "boolean") {
        console.error("message.state.motor_spinning should be a boolean value.");
        return;
    }

    if (!message.state.hasOwnProperty("motor_speed")) {
        console.error("message.state should have 'motor_spped' property.");
        return;
    }

    if (typeof message.state.motor_speed !== "number") {
        console.error("message.state.motor_speed should be a integer value.");
        return;
    }

    if (message.state.speed < 1100 && message.state.speed > 1900) {
        console.error("message.state.motor_speed should be between 1100 and 1900.");
        return;
    }

    ardupilotState.connected = message.state.connected;
    ardupilotState.deviceConnectionString = message.state.device_connection_string;
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

