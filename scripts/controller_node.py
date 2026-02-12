#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Wrench
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class CansatXYControl(Node):
    def __init__(self):
        super().__init__('cansat_xy_control')
        
        
        # Topic : /cansat/camera_down/image_raw
        self.sub_img = self.create_subscription(Image, '/cansat/camera_down/image_raw', self.image_callback, 10)
        
        self.pub_left = self.create_publisher(Wrench, '/cansat/left_arm_cmd', 10)
        self.pub_right = self.create_publisher(Wrench, '/cansat/right_arm_cmd', 10)
        
        # Debug topic : /cansat/debug_image
        self.pub_debug = self.create_publisher(Image, '/cansat/debug_image', 10)
        
        self.bridge = CvBridge()


        # PID Gain
        self.pid_x = {'kp': 1, 'ki': 0.0, 'kd': 0.0, 'integ': 0.0, 'prev': 0.0}
        self.pid_y = {'kp': 1, 'ki': 0.0, 'kd': 0.0, 'integ': 0.0, 'prev': 0.0}
        
        self.last_time = time.time()
        self.max_force = 1.5 
        self.base_brake = 0.5
        
        
        #sequence parameter
        self.state = "IDLE"      # IDLE, ACTION, WAIT
        self.current_direction = "NONE" # LEFT, RIGHT, 

        self.action_start_time = 0.0
        self.wait_duration = 2.0 # 행동 후 기다리는 시간 (초)
        self.action_duration = 0.5 # 줄을 당기고 있는 시간 (초)
        self.turn_force = self.max_force

    def calculate_pid(self, error, pid_dict, dt):
        pid_dict['integ'] += error * dt
        pid_dict['integ'] = max(min(pid_dict['integ'], 5.0), -5.0)
        
        derivative = (error - pid_dict['prev']) / dt
        output = (pid_dict['kp'] * error) + (pid_dict['ki'] * pid_dict['integ']) + (pid_dict['kd'] * derivative)
        
        pid_dict['prev'] = error
        return output

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w, _ = cv_image.shape
            cx, cy = w // 2, h // 2
            
            # Red detect
            blurred = cv2.GaussianBlur(cv_image, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, (0, 70, 50), (10, 255, 255)) | cv2.inRange(hsv, (170, 70, 50), (180, 255, 255))
            mask = cv2.erode(mask, None, iterations=2); mask = cv2.dilate(mask, None, iterations=2)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            err_x_norm = 0.0
            err_y_norm = 0.0
            detected = False
            target_pos = (cx, cy)

            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                M = cv2.moments(c)
                if M["m00"] > 0:
                    tx = int(M["m10"] / M["m00"])
                    ty = int(M["m01"] / M["m00"])
                    target_pos = (tx, ty)
                    detected = True
                    
                    # --- [중요] Error 계산 (정규화 -1.0 ~ 1.0) ---
                    # X error: 물체가 왼쪽에 있으면(+), 왼쪽으로 가야함
                    # Y error: 물체가 위쪽에 있으면(+), 기수를 들어야 함(브레이크)
                    err_x_norm = (cx - tx) / (w / 2) 
                    err_y_norm = (cy - ty) / (-h / 2)
                    
                    print(f"{err_x_norm} , {err_y_norm}")

            curr_time = time.time()
            dt = curr_time - self.last_time
            if dt == 0: dt = 0.001
                        
            turn_effort = self.calculate_pid(err_x_norm, self.pid_x, dt)
            
            brake_effort = self.calculate_pid(err_y_norm, self.pid_y, dt)

            
            left_val = self.base_brake + brake_effort + turn_effort
            right_val = self.base_brake + brake_effort - turn_effort
            
            if not detected:
                self.state = "IDLE"
                self.current_direction = "NONE"
                self.publish_command(0.0, 0.0)
            else:
                
                if self.state == "IDLE":
                        # X, Y 에러 중 절대값이 더 큰 것을 우선순위로 둠 (Dominant Error)
                        abs_x = abs(err_x_norm)
                        abs_y = abs(err_y_norm)
                        threshold = 0.1  # Deadband

                        if abs_x > threshold or abs_y > threshold:
                            self.state = "ACTION"
                            self.action_start_time = curr_time
                            
                            # [TODO 구현] 4-Direction 판별 로직
                            if abs_x >= abs_y:
                                # X축 에러가 더 큼 -> 회전(Turn) 우선
                                if err_x_norm > 0:
                                    self.current_action = "LEFT"
                                else:
                                    self.current_action = "RIGHT"
                            else:
                                # Y축 에러가 더 큼 -> 속도(Pitch) 제어 우선
                                if err_y_norm > 0:
                                    self.current_action = "BRAKE" # 목표가 아래/지나침 -> 감속
                                else:
                                    self.current_action = "GLIDE" # 목표가 위/멀리있음 -> 활공
                                    
                            print(f"[DECISION] ErrX:{err_x_norm:.2f}, ErrY:{err_y_norm:.2f} -> {self.current_action}")

                        else:
                            left_val = 0.0
                            right_val = 0.0

                        
                elif self.state == "ACTION":
                    if curr_time - self.action_start_time <= self.action_duration:
                        
                        force_x = max(0.0, min(raw_pid_x, self.max_force))
                        force_y = max(0.0, min(raw_pid_y, self.max_force))
                        
                        if self.current_action == "LEFT":
                            left_val = force_x
                            right_val = 0.0
                        elif self.current_action == "RIGHT":
                            left_val = 0.0
                            right_val = force_x
                        elif self.current_action == "BRAKE":
                            left_val = -force_y
                            right_val = -force_y
                        elif self.current_action == "GLIDE":
                            left_val = 0.0
                            right_val = 0.0
                    else:
                        self.state = "WAIT"
                        self.action_start_time = curr_time # 타이머 리셋 (Wait용)
                        print(f"[ACTION DONE] Switching to WAIT...")
                        
                        
                elif self.state == "WAIT":
                                
                    left_val = 0
                    right_val = 0
                    
                    if curr_time >= self.wait_duration + self.action_start_time:
                        self.state = "IDLE"
                        #IDLE state -> control하기 위해 거쳐야 함.
                
                self.publish_command(left_val, right_val)
                
                self.last_time = curr_time

            # debug info
            cv2.line(cv_image, (cx, 0), (cx, h), (255, 0, 0), 1)
            cv2.line(cv_image, (0, cy), (w, cy), (255, 0, 0), 1)
            
            if detected:
                cv2.circle(cv_image, target_pos, 8, (0, 255, 0), 2)
                cv2.line(cv_image, (cx, cy), target_pos, (0, 255, 255), 2)
                
                info_text = f"Err X: {err_x_norm:.2f} | Err Y: {err_y_norm:.2f}"
                cv2.putText(cv_image, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            force_text = f"L_Force: {left_val:.1f} | R_Force: {right_val:.1f}"
            cv2.putText(cv_image, force_text, (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            
            
            

        except Exception as e:
            self.get_logger().error(f'Control Error: {e}')

    def publish_command(self, left, right):
        msg_l = Wrench(); msg_l.force.x = left
        msg_r = Wrench(); msg_r.force.x = right
        self.pub_left.publish(msg_l)
        self.pub_right.publish(msg_r)

def main():
    rclpy.init()
    node = CansatXYControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()