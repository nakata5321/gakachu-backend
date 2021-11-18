#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import threading

import uvicorn
from fastapi import FastAPI, Request, Form
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.middleware.cors import CORSMiddleware

# ROS related

threading.Thread(target=lambda: rospy.init_node("frontend", disable_signals=True)).start()

word_publisher = rospy.Publisher("/word_for_gakachu", String, queue_size=1)
dist_dir = rospy.get_param("/frontend/dist")

app = FastAPI()

origins = ["*"]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Mounting default Vue files after running npm run build
app.mount("/dist", StaticFiles(directory=dist_dir), name="dist")
app.mount("/assets", StaticFiles(directory=dist_dir + "assets/"), name="assets")
app.mount("/css", StaticFiles(directory=dist_dir + "assets/css"), name="css")
app.mount("/data", StaticFiles(directory=dist_dir + "assets/data/"), name="data")
app.mount("/Dev", StaticFiles(directory=dist_dir + "assets/data/Dev"), name="Dev")

app.mount("/js", StaticFiles(directory=dist_dir + "assets/js"), name="js")
app.mount("/static", StaticFiles(directory=dist_dir + "assets/static"), name="static")

templates = Jinja2Templates(directory=dist_dir)


@app.get("/", response_class=HTMLResponse)
def root(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})


@app.post("/send_word")
def send_word(word: str = Form(...)):
    word_publisher.publish(String(word))
    return f"Word {word} has been published"


if __name__ == "__main__":
    uvicorn.run("main:app", host="127.0.0.1", port=5000, log_level="info")
