#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import threading

import uvicorn
from typing import Dict, Any
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# ROS related

threading.Thread(
    target=lambda: rospy.init_node("frontend", disable_signals=True)
).start()


def callback(ros_data: String) -> None:
    global status
    check = ros_data.data
    if check == "start":
        status = "busy"
    elif check == "stop":
        status = "available"
    else:
        rospy.logwarn("Error status message. should be 'start' or 'stop'.")


word_publisher = rospy.Publisher("/word_for_gakachu", String, queue_size=1)
color_publisher = rospy.Publisher("/color_height", String, queue_size=1)
test_publisher = rospy.Publisher("/run", String, queue_size=1)
status_listener = rospy.Subscriber("/film", String, callback, queue_size=1)
dist_dir = rospy.get_param("/frontend/dist")

# Fast API related
app = FastAPI()


class Word(BaseModel):
    word: str


origins = ["*"]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Mounting
app.mount("/dist", StaticFiles(directory=dist_dir), name="dist")
app.mount("/loading", StaticFiles(directory=dist_dir + "loading/"), name="loading")
app.mount("/assets", StaticFiles(directory=dist_dir + "assets/"), name="assets")
app.mount("/css", StaticFiles(directory=dist_dir + "assets/css"), name="css")
app.mount("/data", StaticFiles(directory=dist_dir + "assets/data/"), name="data")
app.mount("/Dev", StaticFiles(directory=dist_dir + "assets/data/dev"), name="Dev")
app.mount("/js", StaticFiles(directory=dist_dir + "assets/js"), name="js")
app.mount("/static", StaticFiles(directory=dist_dir + "assets/static"), name="static")

templates = Jinja2Templates(directory=dist_dir)
status = "available"


@app.get("/", response_class=HTMLResponse)
def root(request: Request) -> Any:
    return templates.TemplateResponse("index.html", {"request": request})


@app.get("/Loading", response_class=HTMLResponse)
def loading_page(request: Request) -> Any:
    return templates.TemplateResponse("loading/index.html", {"request": request})


@app.post("/send_word", response_class=JSONResponse)
def send_word(words: Word) -> Dict[str, str]:
    word_publisher.publish(String(words.word))
    return {"status": "OK"}


@app.get("/status", response_class=JSONResponse)
def status_response() -> Dict[str, str]:
    return {"status": status}


@app.get("/brush_lower", response_class=JSONResponse)
def lower() -> Dict[str, str]:
    color_publisher.publish("minus")
    return {"status": "OK"}


@app.get("/brush_raise", response_class=JSONResponse)
def raiser() -> Dict[str, str]:
    color_publisher.publish("plus")
    return {"status": "OK"}


@app.get("/test_brush_position", response_class=JSONResponse)
def test_position() -> Dict[str, str]:
    test_publisher.publish("/home/kuka/kuka_pics/empty.png")
    return {"status": "OK"}


if __name__ == "__main__":
    uvicorn.run("main:app", host="127.0.0.1", port=5000, log_level="debug")
