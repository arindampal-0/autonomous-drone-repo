"""main file"""

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

app = FastAPI()

app.mount("/", StaticFiles(directory="public", html=True), name="static")

@app.get("/health")
def root():
    """health route handler"""
    return {"status": "OK"}
