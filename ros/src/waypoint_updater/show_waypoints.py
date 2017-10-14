#!/usr/bin/python
"""
Visualize the project

"""

import sys
import math
import threading
from PyQt5 import QtGui, QtWidgets
from PyQt5.QtGui import QPen
from PyQt5.QtCore import Qt, QPointF, QRectF, QTimer

import rospy
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import PoseStamped
from dbw_mkz_msgs.msg import SteeringCmd, SteeringReport
from styx_msgs.msg import TrafficLightArray, Lane
from sensor_msgs.msg import Image


class Visualization(QtWidgets.QWidget):

    """
    Subscribe to all the ros publisher and show their content
    """

    def __init__(self):
        super(Visualization, self).__init__()
        rospy.init_node('show_waypoints')
        self.lock = threading.Lock()

        self.waypoints = None
        self.base_waypoints = None
        self.steering = 0
        self.steering_report = None
        self.lights = None
        self.traffic_light = - 1
        self.current_pose = None
        self.dbw_enabled = False

        rospy.Subscriber('/final_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb)
        rospy.Subscriber('/vehicle/steering_cmd', SteeringCmd, self.steering_cb)
        rospy.Subscriber('/vehicle/steering_report', SteeringReport, self.steering_report_cb)
        rospy.Subscriber('/image_color', Image, self.camera_callback)
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_waypoint_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)

        self.img_format_table = {'rgb8': QtGui.QImage.Format_RGB888, 'mono8': QtGui.QImage.Format_Mono}
        self.image = None
        self.initUI()
        self.timer = QTimer()
        self.timer.timeout.connect(self.repaint)
        self.timer.setInterval(1000)
        self.timer.start()

    def initUI(self):
        """"
        Initialize the gui
        """
        self.setGeometry(100, 100, 2000, 1000)
        self.setWindowTitle('Points')
        self.show()

    def paintEvent(self, e):
        """
        Paint all the ocntent
        :param e:
        :return:
        """
        painter = QtGui.QPainter()
        painter.begin(self)
        self.drawPoints(painter)
        painter.end()

    def draw_traffic_lights(self, painter):
        """
        If traffic lights have been provided, draw them.
        :param painter:
        :return:
        """
        if self.lights:
            pen = QPen()
            pen.setWidth(10)
            pen.setColor(Qt.blue)
            painter.setPen(pen)
            for position in self.lights:
                x = position.pose.pose.position.x / 1000 * 400
                y = position.pose.pose.position.y / 1000 * 400 - 400
                painter.drawPoint(x, y)

    def draw_dbw_enabled(self, painter):
        """
        Are we in manual or automatic mode
        :param painter:
        :return:
        """
        pen = QPen()
        pen.setColor(Qt.black)
        painter.setPen(pen)
        text = "Automatic" if self.dbw_enabled else "Manual"
        painter.drawText(QPointF(10,20), text)

    def draw_current_pose(self, painter):
        """
        Draw the current position
        :param painter:
        :return:
        """
        if self.current_pose:
            pen = QPen()
            pen.setWidth(15)
            pen.setColor(Qt.darkMagenta)
            painter.setPen(pen)
            x = self.current_pose.pose.position.x / 1000 * 400
            y = self.current_pose.pose.position.y / 1000 * 400 - 400

            painter.drawPoint(x, y)

    def draw_next_traffic_light(self, painter):
        """
        Draw the upcoming traffic light
        :param painter:
        :return:
        """
        twp = self.traffic_light
        if twp >= 0 and self.lights:
            pen = QPen()
            pen.setWidth(20)
            pen.setColor(Qt.red)
            painter.setPen(pen)
            x = self.base_waypoints[twp].pose.pose.position.x / 1000 * 400
            y = self.base_waypoints[twp].pose.pose.position.y / 1000 * 400 - 400
            painter.drawPoint(x, y)

    def drawPoints(self, painter):
        """
        Draw the recevied content
        :param painter:
        :return:
        """
        pen = QPen()
        pen.setWidth(4)
        pen.setColor(Qt.black)
        painter.setPen(pen)
        if self.base_waypoints:
            for waypoint in self.base_waypoints:
                x = waypoint.pose.pose.position.x / 1000 * 400
                y = waypoint.pose.pose.position.y / 1000 * 400 - 400
                painter.drawPoint(x, y)
        pen = QPen()
        pen.setWidth(6)
        pen.setColor(Qt.red)
        painter.setPen(pen)
        if self.waypoints:
            for waypoint in self.waypoints:
                x = waypoint.pose.pose.position.x / 1000 * 400
                y = waypoint.pose.pose.position.y / 1000 * 400 - 400
                painter.drawPoint(x, y)

        cx = 500
        cy = 500
        r = 100.0
        pen = QPen()
        pen.setWidth(3)
        pen.setColor(Qt.black)
        painter.setPen(pen)
        painter.drawEllipse(QPointF(cx, cy), r, r)

        self.draw_steering(painter, cx, cy, r, 10, self.steering, Qt.red)
        self.draw_steering_report(painter, cx, cy, r, Qt.blue)

        if self.image:
            painter.drawImage(QRectF(1000, 200, self.image.size().width(), self.image.size().height()), self.image)

        self.draw_next_traffic_light(painter)
        self.draw_dbw_enabled(painter)
        self.draw_current_pose(painter)
        self.draw_traffic_lights(painter)

    def draw_steering(self, painter, cx, cy, r, width, steering, color):
        pen = QPen()
        pen.setWidth(width)
        pen.setColor(color)
        painter.setPen(pen)
        x = cx + r * math.cos(-math.pi / 2 + steering * -1)
        y = cy + r * math.sin(-math.pi / 2 + steering * -1)
        painter.drawLine(QPointF(cx, cy), QPointF(x, y))

    def draw_steering_report(self, painter, cx, cy, r, color):
        if self.steering_report:
            pen = QPen()
            pen.setColor(Qt.black)
            painter.setPen(pen)
            text = "%4d km/h" % (self.steering_report.speed*3.6)
            painter.drawText(QPointF(cx, cy-r-40), text)

            self.draw_steering(painter, cx, cy, r, 5, self.steering_report.steering_wheel_angle, color)

    def waypoints_cb(self, msg):
        """
        Callback for /final_waypoints
        :param msg:
        :return:
        """
        self.waypoints = msg.waypoints

    def steering_cb(self, msg):
        """
        Callback for /vehicle/steering_cmd
        :param msg:
        :return:
        """
        self.steering = msg.steering_wheel_angle_cmd

    def steering_report_cb(self, msg):
        """
        Callback for /vehicle/steering_cmd
        :param msg:
        :return:
        """
        self.steering_report = msg

    def base_waypoints_cb(self, msg):
        """
        Callback for /base_waypoints
        :param msg:
        :return:
        """
        self.base_waypoints = msg.waypoints

    def camera_callback(self, data):
        """
        Callback for /image_color
        :param data:
        :return:
        """
        _format = self.img_format_table[data.encoding]

        image = QtGui.QImage(data.data, data.width, data.height, _format)
        self.image = image

    def traffic_cb(self, msg):
        """
        Callback for /vehicle/traffic_lights
        :param msg:
        :return:
        """
        self.lights = msg.lights

    def traffic_waypoint_cb(self, msg):
        """
        Callback for /traffic_waypoint
        :param msg:
        :return:
        """
        self.traffic_light = msg.data

    def current_pose_cb(self, msg):
        """
        Callback for /traffic_waypoint
        :param msg:
        :return:
        """
        self.current_pose = msg

    def dbw_enabled_cb(self, msg):
        """
        Callback for /vehicle/dbw_enable
        :param msg:
        :return:
        """
        self.dbw_enabled = msg.data

def main():
    app = QtWidgets.QApplication(sys.argv)
    Visualization()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
