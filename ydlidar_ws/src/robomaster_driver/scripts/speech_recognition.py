#!/usr/bin/python3
# coding=utf8
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray
import smbus
import time
import subprocess
from robomaster import robot
from robomaster import conn


# I2C地址
I2C_ADDR = 0x34  # I2C地址
ASR_RESULT_ADDR = 100  # ASR结果寄存器地址
ASR_SPEAK_ADDR = 110  # ASR说话寄存器地址
ASR_CMDMAND = 0x00
ASR_ANNOUNCER = 0xFF

start_pose=None
goal_pub=None
not_start = True

last_status = None
state = "idle"   # 可选值："idle"、"going1"、"returning"

status=0
num1=0
num2=0
count=0

class ASRModule:
    def __init__(self,address, bus=1):
        # 初始化 I2C 总线和设备地址
        self.bus = smbus.SMBus(bus)  # 使用 I2C 总线 1
        self.address = address  # 设备的 I2C 地址
        self.send = [0, 0]  # 初始化发送数据的数组

    def wire_write_byte(self, val):
        """
        向设备写入单个字节
        :param val: 要写入的字节值
        :return: 如果成功写入返回 True，失败返回 False
        """
        try:
            self.bus.write_byte(self.address, val) # 发送字节到设备
            return True # 写入成功
        except IOError:
            return False # 写入失败，返回 False

    def wire_write_data_array(self, reg, val, length):
        """
        向指定寄存器写入字节数组
        :param reg: 寄存器地址
        :param val: 要写入的字节数组
        :param length: 要写入的字节数
        :return: 如果成功写入返回 True，失败返回 False
        """
        try:            
            self.bus.write_i2c_block_data(self.address, reg, val[:length]) # 发送字节数组到设备的指定寄存器
            return True # 写入成功
        except IOError:
            return False # 写入失败，返回 False

    def wire_read_data_array(self, reg, length):
        """
        从指定寄存器读取字节数组
        :param reg: 寄存器地址
        :param length: 要读取的字节数
        :return: 读取到的字节数组，失败时返回空数组
        """          
        try:
            result = self.bus.read_i2c_block_data(self.address, reg, 1) # 从设备读取字节数组
            return result # 返回读取结果
        except IOError:
            return [] # 读取失败，返回空数组

    def rec_recognition(self):
        """
        识别结果读取
        :return: 识别结果，如果读取失败返回 0
        """
        result = self.wire_read_data_array(ASR_RESULT_ADDR, 1) # 从结果寄存器读取一个字节
        if result:
            return result # 返回读取到的结果
        return 0  # 如果没有结果，返回 0

    def speak(self, cmd, id):
        """
        向设备发送说话命令
        :param cmd: 命令字节
        :param id: 说话的 ID
        """
        if cmd == ASR_ANNOUNCER or cmd == ASR_CMDMAND: # 检查命令是否有效
            self.send[0] = cmd # 设置发送数组的第一个元素为命令
            self.send[1] = id # 设置发送数组的第二个元素为 ID
            self.wire_write_data_array(ASR_SPEAK_ADDR, self.send, 2) # 发送命令和 ID 到指定寄存器

def amcl_callback(msg):
    global start_pose
    if start_pose is None:
        start_pose = PoseStamped()
        start_pose.header.frame_id = "map"
        start_pose.pose = msg.pose.pose
        rospy.loginfo(" 初始位置已记录")

def publish_start_pose():
    global start_pose, goal_pub, not_start
    if start_pose is not None and not_start:
        start_pose.header.stamp = rospy.Time.now()
        goal_pub.publish(start_pose)
        rospy.loginfo("返回起点")
        not_start = False

def status_callback(msg):
    global last_status, state, start_pose, goal_pub, asr_module, not_start

    # 如果没有状态列表，直接返回
    if not msg.status_list:
        return

    # 取最新的状态码
    current = msg.status_list[-1].status

    # **仅在状态从非 3 → 3 的“上升沿”时触发一次到达逻辑**
    if current == 3 and last_status != 3:
        rospy.loginfo("到达目标状态(3) 检测到上升沿")

        if state == "going1":
            # 到达点1
            asr_module.speak(ASR_ANNOUNCER, 2)
            state = "returning"
            not_start = True
            # 延迟 2 秒发布回起点命令
            rospy.Timer(
                rospy.Duration(5.0),
                lambda event: publish_start_pose(),
                oneshot=True
            )

        elif state == "returning":
            # 回到起点
            asr_module.speak(ASR_ANNOUNCER, 3)
            state = "idle"

    # 更新上一次状态，用于下次上升沿检测
    last_status = current


def send_goal(x, y, z,w):
    global goal_pub,num1
    num1 =0
    rospy.loginfo(f"diaoyong")
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.z = z
    goal.pose.orientation.w = w
    goal_pub.publish(goal)
    rospy.loginfo(f" 已发布目标点: x={x}, y={y}, z={z}, w={w}")

if __name__ == "__main__":
    rospy.init_node('speech_nav_node')
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_callback)
    rospy.Subscriber("/move_base/status", GoalStatusArray, status_callback)

    asr_module = ASRModule(I2C_ADDR)
    rospy.sleep(1.0)
    rospy.loginfo(" 开始语音识别导航...")
    # speak delivery start
    asr_module.speak(ASR_ANNOUNCER, 1)

    while not rospy.is_shutdown():
        recognition_result = asr_module.rec_recognition()
        if recognition_result[0] != 0:  
            rospy.loginfo(f"🗣 语音识别编号: {recognition_result[0]}")
            if recognition_result[0] == 1:
                #speak ready to go
                state = "going1"
                send_goal( -2.80978822708,  -5.84721469879, 0.0135779660091, 0.999907815171) # 点1
            elif recognition_result[0] == 2:
                #speak ready to go
                state = "going1"
                send_goal(-0.569830775261,-9.50584697723, -0.734510563518, 0.678597253223) # 点2


# if __name__ == "__main__":
#     asr_module = ASRModule(I2C_ADDR)    
#     pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
#     rospy.init_node('send_goal_node')
#     while True:
#         recognition_result = asr_module.rec_recognition()
#         if recognition_result[0] != 0:
#             if recognition_result[0] == 1:
#                 goal = PoseStamped()
#                 goal.header.frame_id = "map"
#                 goal.header.stamp = rospy.Time.now()
#                 goal.pose.position.x = -3.11439418793
#                 goal.pose.position.y = -6.01108407974
#                 goal.pose.orientation.w=0.027156766503971863
#                 pub.publish(goal)
#                 #subprocess.Popen(["roslaunch","robomaster_driver","start.launch"])
#                 #asr_module.speak(ASR_ANNOUNCER, 1)
#             elif recognition_result[0] == 2:
#                 goal = PoseStamped()
#                 goal.header.frame_id = "map"
#                 goal.header.stamp = rospy.Time.now()
#                 goal.pose.position.x = -0.569830775261
#                 goal.pose.position.y = -9.5478553772
#                 goal.pose.orientation.w=-1.649890303612984
#                 pub.publish(goal)
#             elif recognition_result[0] == 3:
#                 print("left")
#             elif recognition_result[0] == 4:
#                 print("right")
#             elif recognition_result[0] == 9:
#                 print("stop")