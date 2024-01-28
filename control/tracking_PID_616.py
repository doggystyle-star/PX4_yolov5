import cv2
import math
import rospy
import random
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from pyquaternion import Quaternion
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist, PoseStamped

import sys 
sys.path.append('/home/robot/firmware/catkin_ws/devel/lib/python3/dist-packages')

class PIDController():
    def __init__(self, kp, ki, kd, max, min):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error_sum = 0.0
        self.last_error = 0.0
        self.max = max
        self.min = min
    def update(self, target, current, dt):
        error  = target-current
        self.error_sum += error * dt
        error_diff = (error - self.last_error)/dt
        #error_diff = (error - self.last_error)
        output = self.kp * error +self.ki * self.error_sum + self.kd * error_diff
        self.last_error = error

        return output
# 饱和函数（saturation function）
def sat(x,u):
    if x < -u:
        return -u
    elif x > u:
        return u
    else:
        return x 

# 双曲正切函数（tanh function）
def tanh(x):
    return np.tanh(x)

class AccelerateController():
    def __init__(self, tao1, tao2, Et,Ua=0.0):
        self.tao1 = tao1
        self.tao2 = tao2
        self.Et = Et
        self.Sv = 0
        self.Su = 0
        self.Ua = Ua

    def update_X(self,U_,dt,VL,PL):
        #delta_Ua = -self.Et*(self.Sv - VL) + self.Su
        #U = -1*self.tao2*(tanh(VL + self.tao1*PL)+tanh(VL)) - sat(delta_Ua,U_)
        U = -1*self.tao2*(tanh(VL + self.tao1*PL)+tanh(VL))
        acc = U
        #acc = U + self.Ua
        V_out = VL + acc*dt
        #更新Sv和Su
        Sv_1 = self.Sv - self.Et*(self.Sv - VL)*dt
        Su_1 = self.Su - self.Et*(self.Su + U)*dt
        self.Sv = Sv_1
        self.Su = Su_1
        return V_out
    def update_Y(self,U_,dt,VL,PL):
        delta_Ua = -1*self.Et*(self.Sv - VL) + self.Su
        U =-self.tao2*(tanh(VL + self.tao1*PL)+tanh(VL)) - sat(delta_Ua,U_)
        acc = U
        V_out = VL + acc*dt
        #更新Sv和Su
        Sv_1 = self.Sv - self.Et*(self.Sv - VL)*dt
        Su_1 = self.Su - self.Et*(self.Su + U - 9.8)*dt
        self.Sv = Sv_1
        self.Su = Su_1
        return V_out
    def update_Z(self,U_,dt,VL,PL):
        delta_Ua = -1*self.Et*(self.Sv - VL) + self.Su
        U = 9.8 - self.tao2*(tanh(VL + self.tao1*PL)+tanh(VL)) - sat(delta_Ua,U_)
        acc = U - 9.8
        V_out = VL + acc*dt
        #更新Sv和Su
        Sv_1 = self.Sv - self.Et*(self.Sv - VL)*dt
        Su_1 = self.Su - self.Et*(self.Su + U - 9.8)*dt
        self.Sv = Sv_1
        self.Su = Su_1
        return V_out
bridge = CvBridge()

def color_img_callback(msg):
    global color_img
    color_img = bridge.imgmsg_to_cv2(msg, "bgr8")

def depth_img_callback(msg):
    global depth_img
    depth_img = bridge.imgmsg_to_cv2(msg, "32FC1")
    depth_img = np.nan_to_num(depth_img)

def detact_distance(box,randnum):
    global color_img, depth_img
    distance_list = []
    mid_pos = [(box.xmin + box.xmax)//2, (box.ymin + box.ymax)//2] #确定索引深度的中心像素位置
    min_val = min(abs(box.xmax - box.xmin), abs(box.ymax - box.ymin))#确定深度搜索范围
    #print(box,)
    for i in range(randnum):
        bias = random.randint(-min_val//4, min_val//4)
        dist = depth_img[int(mid_pos[1] + bias), int(mid_pos[0] + bias)]
        #cv2.circle(color_img (int(mid_pos[0] + bias), int(mid_pos[1] + bias)), 4, (255,0,0), -1)
        #print(int(mid_pos[1] + bias), int(mid_pos[0] + bias))
        if dist:
            distance_list.append(dist)
    distance_list = np.array(distance_list)
    distance_list = np.sort(distance_list)[randnum//2-randnum//4:randnum//2+randnum//4] #冒泡排序+中值滤波
    #print(distance_list, np.mean(distance_list))
    return np.mean(distance_list)
        
def local_pose_callback(msg):
    global height, target_height, target_set
    height = msg.pose.position.z 
    z_velocity = PIDController(kp = Kp_z, ki = Ki_z, kd = Kd_z,max = 0.5, min= -0.5)
    twist.linear.z = z_velocity.update(target= target_height, current= msg.pose.position.z,dt = Dt)
     
    if not target_set:
        target_height = height     
        target_set = True        

def darknet_callback(data):
    global find_cnt, twist, cmd, target_height,get_time,eval_distance
    for box in data.bounding_boxes:
        if(box.id == 0 ):
            print('find human')
            eval_distance = detact_distance(box,24)
            print(eval_distance)
            u = (box.xmax+box.xmin)/2
            v = (box.ymax+box.ymin)/2
            print(u)
            toast.linear.x = u - u_center
            toast.linear.y = v - v_center
            toast.linear.z = eval_distance
            z_angvelocity = PIDController(kp = Kp_yaw, ki = Ki_yaw, kd = Kd_yaw,max = 0.5, min= -0.5)
            twist.angular.z = z_angvelocity.update(u_center,u,Dt)

            x_velocity = PIDController(kp = Kp_x, ki = Ki_x, kd = Kd_x,max = 0.5, min= -0.5)
            twist.linear.x = x_velocity.update(eval_distance,target_distance,Dt)
            
            # y_velocity = PIDController(kp = Kp_y, ki = Ki_y, kd = Kd_y,max = 0.05, min= -0.05)
            # twist.linear.y = y_velocity.update(v_center,v,Dt)
            cmd = ''
            find_cnt = find_cnt + 1
            get_time = False
            print(twist.angular.z)


if __name__ == "__main__":
    target_distance = 4
    target_height = 1.0

    u_center=1080/2 
    v_center=720/2

    #PID control
    #Kp_x = -0.0
    Kp_x = 0.2
    Ki_x = 0
    #Kd_x = -0.0
    Kd_x = 0.0001

    Kp_yaw = 0.0005
    Ki_yaw = 0.01
    Kd_yaw = 0.00002

    Kp_z = 0.3
    Ki_z = 0
    Kd_z = 0.01

    Dt = 0.01
    target_set = True
    find_cnt_last = 0
    not_find_time = 0
    get_time = False
    twist = Twist()
    toast = Twist()
    cmd = String()
    find_cnt = 0
    height = 0  
    #fx = 205.46963709898583
    #fy = 205.46963709898583
    #fx = 385.7544860839844
    #fy = 385.7544860839844
    fx = 616.591
    fy = 616.765


    vehicle_type = sys.argv[1]
    vehicle_id = sys.argv[2]
    rospy.init_node('yolo_human_tracking')

    rospy.Subscriber(vehicle_type+'_'+vehicle_id+"realsense/depth_camera/color/image_raw",Image,callback = color_img_callback, queue_size=1)

    rospy.Subscriber(vehicle_type+'_'+vehicle_id+"/realsense/depth_camera/depth/image_raw", Image, callback = depth_img_callback, queue_size=1)

    rospy.Subscriber("/uav_"+vehicle_id+"/darknet_ros/bounding_boxes", BoundingBoxes, callback = darknet_callback,queue_size=1)

    rospy.Subscriber(vehicle_type+'_'+vehicle_id+"/mavros/local_position/pose", PoseStamped, local_pose_callback,queue_size=1)

    cmd_vel_pub = rospy.Publisher('/xtdrone/'+vehicle_type+'_'+vehicle_id+'/cmd_vel_flu', Twist, queue_size=1)
    cmd_pub = rospy.Publisher('/xtdrone/'+vehicle_type+'_'+vehicle_id+'/cmd', String, queue_size=1)
    error_pub = rospy.Publisher('/xtdrone/'+vehicle_type+'_'+vehicle_id+'/cmd_error',Twist,queue_size=1)
    rate = rospy.Rate(60) 


    while not rospy.is_shutdown():
        rate.sleep()
        cmd_vel_pub.publish(twist)
        error_pub.publish(toast)
        cmd_pub.publish(cmd)
        if find_cnt - find_cnt_last == 0:
            if not get_time:
                not_find_time = rospy.get_time()
                get_time = True
            if (rospy.get_time() - not_find_time > 3.0)or eval_distance =='nan':
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                cmd = 'HOVER'
                print(cmd)
                
                get_time = False
        find_cnt_last = find_cnt
