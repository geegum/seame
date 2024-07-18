#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from datetime import datetime

MAX_SPEED = 0.25
pub_steering = rospy.Publisher('/Steering', Float32, queue_size=1)
pub_throttle = rospy.Publisher('/Throttle', Float32, queue_size=1)
file_path = "/home/jetson/catkin_ws/video/teleop_data.txt"

def callback_joy(joy_msg):
    steering = -joy_msg.axes[2]
    throttle = joy_msg.axes[1]
    rospy.loginfo("steering: %f", steering)
    rospy.loginfo("throttle: %f", throttle)
    
    pub_steering.publish(steering)
    if throttle < 0:
        weight = MAX_SPEED * 1.3
    else:
        weight = MAX_SPEED
    pub_throttle.publish(throttle * weight)
    
    # 파일에 데이터 저장
    with open(file_path, "a") as file:
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        file.write(f"{current_time}, steering: {steering}, throttle: {throttle}\n")

def main():
    rospy.init_node('teleop', anonymous=False)
    rospy.loginfo('teleop node initiated!')
    rospy.Subscriber("/joy", Joy, callback_joy)
    rospy.spin()

if __name__ == "__main__":
    main()
