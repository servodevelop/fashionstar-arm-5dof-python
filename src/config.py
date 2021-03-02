'''
Fashion Star 5DoF机械臂的配置文件
* 作者: 阿凯
* Email: kyle.xing@fashionstar.com.hk
* 更新时间: 2021/03/02
'''
import math
##########################################
## 机械臂-设备
##
##########################################
# 串口舵机的默认设备号
DEVICE_PORT_DEFAULT = '/dev/ttyUSB0' # RaspberryPi4
# DEVICE_PORT_DEFAULT = '/dev/ttyS0' # RaspberryPi4
# DEVICE_PORT_DEFAULT = 'COM3' # PC Windows
# DEVICE_PORT_DEFAULT = '/dev/ttyUSB0' # PC Ubuntu

##########################################
## 微型气泵与电磁阀
## 适用于FashionStar树莓派串口总线舵机拓展版
##########################################
GPIO_PUMP_MOTOR = 7 # 微型气泵马达在拓展板上的GPIO
GPIO_MAGNETIC_SWITCH = 16 # 电磁阀开关的GPIO

##########################################
## 串口舵机参数
##
##########################################
SERVO_NUM = 5 # 舵机的个数
SERVO_SPEED_DEFAULT = 100 # 舵机默认的平均转速 °/s
SERVO_SPEED_MIN = 1 # 转速的最大值
SERVO_SPEED_MAX = 200 # 转速的最小值

##########################################
## 机械臂-连杆参数
## 单位cm
##########################################
LINK23 = 8.0    # JOINT2与JOINT3(腕关节)的长度
LINK34 = 7.6    # JOINT3与JOINT4(腕关节)的长度       
LINK45 = 13.6    # JOINT4与JOINT5(腕关节)的长度 
# TOOL_LEN = 4.6  # 工具的长度(气泵连杆的长度)

##########################################
## 机械臂-关节设置
##
##########################################
# 关节的编号, 也是关节对应的串口总线舵机的ID号
# 注: JNT是Joint的缩写
JOINT1 = 0
JOINT2 = 1
JOINT3 = 2
JOINT4 = 3
GRIPPER = 4

# 关节弧度的范围
THETA_LOWERB = [-math.pi/2, -3*math.pi/4, 0, -math.pi/2, 0]
THETA_UPPERB = [math.pi/2, 0, 3*math.pi/4, math.pi/2, math.pi/2]
# 机械臂初始化位姿的弧度
THETA_INIT_POSE = {
    JOINT1:0, 
    JOINT2:-3*math.pi/4,
    JOINT3:math.pi/2,
    JOINT4:math.pi/4,
    GRIPPER: 0
}

# 舵机原始角度与关节弧度转换对应的偏移量与比例系数
JOINT2SERVO_K=[-50.930, 48.383, -49.656, -49.656, 57.296]
JOINT2SERVO_B=[0.000, 75.000, 40.000, 0.000, 0.000]
##########################################
## 轨迹规划
##
##########################################
TRAJECTORY_DELTA_T = 0.002 # 单位s

##########################################
## 插补算法
##
##########################################
RAD_PER_STEP = 0.05
