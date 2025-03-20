"""main file"""

import uvicorn
from fastapi import FastAPI, WebSocket
from fastapi.staticfiles import StaticFiles

app = FastAPI(title="app")

api = FastAPI(title="api")


@api.get("/health")
def health_route():
    """health route handler"""
    return {"status": "OK"}


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """websocket handler"""
    await websocket.accept()
    while True:
        data = await websocket.receive_text()
        print(f"Message text was: {data}")
        await websocket.send_text(f"Message text was: {data}")


app.mount("/api", api, name="api")

app.mount("/", StaticFiles(directory="public", html=True), name="static")


def main():
    """main function"""
    uvicorn.run(app, host="0.0.0.0", port=8000)


if __name__ == "__main__":
    main()
