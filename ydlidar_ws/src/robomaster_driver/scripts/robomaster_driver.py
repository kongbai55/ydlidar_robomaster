#!/usr/bin/env python3
# coding: utf-8

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from robomaster import robot
import threading
import time
import math

class RoboMasterDriver:
    def __init__(self):
        rospy.init_node("robomaster_driver", anonymous=False)

        # 初始化 RoboMaster SDK
        self.ep_robot = robot.Robot()
        self.ep_robot.initialize(conn_type="rndis")  
        self.chassis = self.ep_robot.chassis

        # 发布器
        self.vel_pub = rospy.Publisher("/pub_vel", Twist, queue_size=10)
        self.imu_pub = rospy.Publisher("/pub_imu", Imu, queue_size=10)

        # 订阅器
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        # 开启 IMU 数据回调线程
        self.chassis.sub_imu(freq=10, callback=self.imu_callback)

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("RoboMaster ROS driver with IMU started.")

    # def cmd_vel_callback(self, msg):
    #     vx = msg.linear.x
    #     vy = msg.linear.y
    #     wz = msg.angular.z
    #     self.chassis.move(x=vx, y=vy, z=wz, xy_speed=0.5)

    #     # 发布回显速度信息（模拟反馈）
    #     twist = Twist()
    #     twist.linear.x = vx
    #     twist.linear.y = vy
    #     twist.angular.z = wz
    #     self.vel_pub.publish(twist)

    def cmd_vel_callback(self, msg):
        rospy.loginfo("Received /cmd_vel command.")

        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        wz_deg = math.degrees(wz)
        # 尝试控制底盘移动
        try:
            rospy.loginfo("Attempting to send move command: vx=%.2f, vy=%.2f, wz=%.2f", vx, vy, wz)
            self.chassis.drive_speed(x=vx, y=vy, z=-wz_deg,)
            rospy.loginfo("Move command sent successfully.")
        except Exception as e:
           rospy.logerr("Failed to send move command: %s", e)

        # 即使移动失败，也尝试回显
        # try:
        #     twist = Twist()
        #     twist.linear.x = vx
        #     twist.linear.y = vy
        #     twist.angular.z = wz 
        #     self.vel_pub.publish(twist)
        #     rospy.loginfo("Published /pub_vel.")
        # except Exception as e:
        #     rospy.logerr("Failed to publish /pub_vel: %s", e)


    def imu_callback(self, imu_data):
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"

        # 加速度（单位 m/s²）
        msg.linear_acceleration.x = imu_data[0]
        msg.linear_acceleration.y = imu_data[1]
        msg.linear_acceleration.z = imu_data[2]

        # 角速度（单位 rad/s）
        msg.angular_velocity.x = imu_data[3]
        msg.angular_velocity.y = imu_data[4]
        msg.angular_velocity.z = imu_data[5]

        # 姿态（IMU 无法提供 orientation 四元数，若有需要请加上 filter 或保留默认值）
        # msg.orientation.x = ...
        # msg.orientation.y = ...
        # msg.orientation.z = ...
        # msg.orientation.w = ...

        self.imu_pub.publish(msg)

    def shutdown(self):
        rospy.loginfo("Shutting down RoboMaster...")
        self.chassis.unsub_imu()
        self.ep_robot.close()

if __name__ == '__main__':
    try:
        driver = RoboMasterDriver()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Final!!!")
