"""main file"""

from fastapi import FastAPI

app = FastAPI()


@app.get("/")
def root():
    """root handler"""
    return {"msg": "Hello, World!"}
