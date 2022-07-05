import rclpy
from deepracer_interfaces_pkg.msg import ServoCtrlMsg

class MotionControls:
    def __init__(self):
        rclpy.init()
        node = rclpy.create_node("manual_drive_publisher")
        self.publisher = node.create_publisher(ServoCtrlMsg,"/ctrl_pkg/servo_msg",10)

    def stop(self):
        msg = ServoCtrlMsg(angle=0.0, throttle=0.0)
        self.publisher.publish(msg)

    def drive(self, angle, throttle):
        if throttle!=0:
            throttle = 1 # deepracer won't move when throttle<1
        msg = ServoCtrlMsg(angle=float(angle), throttle=float(throttle))
        self.publisher.publish(msg)

    def __del__(self):
        self.stop()
