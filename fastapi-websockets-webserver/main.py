"""main file"""

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

app = FastAPI(title="app")

api = FastAPI(title="api")


@api.get("/health")
def health_route():
    """health route handler"""
    return {"status": "OK"}


app.mount("/api", api, name="api")

app.mount("/", StaticFiles(directory="public", html=True), name="static")
