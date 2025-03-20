console.log("FastAPI WebSocket server")

const ws = new WebSocket("/ws")

const inputField = document.querySelector(".input input[name='text']");
// console.log(inputField);

const form = document.querySelector("form");
// console.log(form);

if (form instanceof HTMLFormElement) {
    form.addEventListener("submit", function(event) {
        event.preventDefault();

        if (inputField instanceof HTMLInputElement) {
            if (inputField.value.length == 0) {
                throw new Error("message should not be empty.");
            } else {
                console.log("Message is:", inputField.value);
                if (ws.OPEN) {
                    ws.send(inputField.value);
                }
            }
        }
    });
}


ws.addEventListener("open", function(event) {
    console.log("Websocket connection opened!");
});

ws.addEventListener("message", function(event) {
    console.log("message: ", event.data);
});

ws.addEventListener("error", function(event) {
    console.error("Websocket errored: ", event.error);
});

ws.addEventListener("close", function(event) {
    console.log("Websocket connection closed!");
});

