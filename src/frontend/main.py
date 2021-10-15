#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String

import threading

import uvicorn

from fastapi import FastAPI

# ROS related

threading.Thread(target=lambda: rospy.init_node("frontend", disable_signals=True)).start()

hello_publisher = rospy.Publisher("/hello_world", String, queue_size=1)

app = FastAPI()

@app.get("/")
def read_root():
    hello_publisher.publish(String("Hello world"))
    return "Hello World"


if __name__ == "__main__":
    uvicorn.run("main:app", host="127.0.0.1", port=5000, log_level="info")

