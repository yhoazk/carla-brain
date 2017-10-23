#!/usr/bin/env python

import sys
import threading
import os
from timeit import default_timer as timer

import signal
import socketio
import eventlet
import eventlet.wsgi
import time
from flask import Flask, render_template
import rospy

from bridge import Bridge
from conf import conf

dbw_enable = False
MONKEY_PATCH = rospy.get_param('do_monkey_patch', False)

if not MONKEY_PATCH:
    sio = socketio.Server()
else:
    eventlet.monkey_patch()
    sio = socketio.Server(async_mode='eventlet')

rospy.init_node('styx_server')
rospy.logwarn("monkey_patch: %r", MONKEY_PATCH)

app = Flask(__name__)
msgs = {}

@sio.on('connect')
def connect(sid, environ):
    rospy.loginfo("connect: %r", sid)

@sio.on('disconnect')
def disconnect(sid):
    global dbw_enable
    rospy.loginfo("disconnect: %r", sid)
    dbw_enable = False
    bridge.publish_dbw_status(dbw_enable)

def send(topic, data):
    s = 1
    msgs[topic] = data
    # sio.emit(topic, data=json.dumps(data), skip_sid=True)

bridge = Bridge(conf, send)


@sio.on('telemetry')
def telemetry(sid, data):
    global dbw_enable
    if data["dbw_enable"] != dbw_enable:
        dbw_enable = data["dbw_enable"]
        bridge.publish_dbw_status(dbw_enable)
    bridge.publish_odometry(data)
    for i in range(len(msgs)):
        topic, data = msgs.popitem()
        sio.emit(topic, data=data, skip_sid=True)


@sio.on('control')
def control(sid, data):
    bridge.publish_controls(data)


@sio.on('obstacle')
def obstacle(sid, data):
    bridge.publish_obstacles(data)


@sio.on('lidar')
def obstacle(sid, data):
    bridge.publish_lidar(data)


@sio.on('trafficlights')
def trafficlights(sid, data):
    bridge.publish_traffic(data)



prev_time = timer()

@sio.on('image')
def image(sid, data):
    global prev_time
    if int((timer() - prev_time) * 1000)>=100:
        # publish image every 100ms at most
        bridge.publish_camera(data)
        prev_time = timer()


def spinning_worker():
    rospy.spin()
    os.kill(os.getpid(), signal.SIGKILL)


if __name__ == '__main__':
    t = threading.Thread(target=spinning_worker)
    t.start()

    # wrap Flask application with engineio's middleware
    app.wsgi_app = socketio.Middleware(sio, app.wsgi_app)

    # deploy as an eventlet WSGI server
    while not rospy.is_shutdown():
        try:
            eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
        except:
            # Until https://github.com/eventlet/eventlet/issues/222 is solved
            rospy.logerr("Unexpected error: %r", sys.exc_info()[0])
