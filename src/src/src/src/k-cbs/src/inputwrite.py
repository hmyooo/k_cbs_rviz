#!/home/rancho/0Triplez/.venv/triplez/bin/python3.8
import rospy
import cv2
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32,PoseStamped
import numpy as np
import yaml
import os,math
global num
Agent_model = {
    "type": "Ackerman",
    "lf": 2,
    "lb": 0.65,
    "width": 1.75,
    "r": 6,
    "delta": 0.11781,
}
data = {}

def ToEulerAngles(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
 
    sinp = sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if math.fabs(sinp) >= 1:
        pitch = math.copysign(M_PI / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0  * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
 
    return yaw


def msg_Callback(point):
    global num
    if num % 2 == 0:
        Agent = {}
        Agent["model"] =dict(Agent_model)
        Agent["index"] = int(num / 2)
        yaw = ToEulerAngles(point.pose.orientation) 
        Agent["start"] = [point.pose.position.x,point.pose.position.y,yaw]   
        data["agents"].append(Agent)
    else:
        data["agents"][int(num/2)]["goal"] = [point.pose.position.x,point.pose.position.y,ToEulerAngles(point.pose.orientation)]
    num+=1
        
    


if __name__ == "__main__":
    rospy.init_node("write_pub")
    file_path = os.path.dirname(os.path.abspath(__file__))
    outputfile =rospy.get_param('~outputfile',file_path + '/input.yaml')
    agv_count =rospy.get_param('~agent_num',10)
    data["agents"] = []
    num = 0
    sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, msg_Callback)
    while not rospy.is_shutdown():
        if num >= agv_count * 2:
            break
        rospy.spin()
    with open(outputfile, "w", encoding="utf-8") as f:
        yaml.dump(data, f, allow_unicode=True)

