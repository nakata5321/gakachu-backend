#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String

import threading

import uvicorn

from fastapi import FastAPI, Form
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

from starlette.responses import FileResponse

# ROS related

threading.Thread(target=lambda: rospy.init_node("frontend", disable_signals=True)).start()

word_publisher = rospy.Publisher("/word_for_gakachu", String, queue_size=1)
assets_dir = rospy.get_param("/frontend/assets")

app = FastAPI()
#app.mount(assets_dir, StaticFiles(directory="assets"), name="assets")
app.mount("/assets", StaticFiles(directory=assets_dir + "assets"), name="assets")

@app.get("/", response_class=FileResponse)
def read_root():
    
    return FileResponse("index.html")

@app.post("/send_word")
def send_word(word: str = Form(...)):
    word_publisher.publish(String(word))
    return f"Word {word} has been published"


if __name__ == "__main__":
    uvicorn.run("main:app", host="127.0.0.1", port=5000, log_level="info")

