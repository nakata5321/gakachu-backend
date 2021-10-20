#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String

import threading

import uvicorn

from fastapi import FastAPI, Form
from fastapi.responses import HTMLResponse

# ROS related

threading.Thread(target=lambda: rospy.init_node("frontend", disable_signals=True)).start()

word_publisher = rospy.Publisher("/word_for_gakachu", String, queue_size=1)
assets_dir = rospy.get_param("/frontend/assets")

app = FastAPI()

@app.get("/", response_class=HTMLResponse)
def read_root():
    content = open(assets_dir + "index.html").read()
    return content

@app.post("/send_word")
def send_word(word: str = Form(...)):
    word_publisher.publish(String(word))
    return f"Word {word} has been published"


if __name__ == "__main__":
    uvicorn.run("main:app", host="127.0.0.1", port=5000, log_level="info")

