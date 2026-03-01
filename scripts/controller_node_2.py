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
        
        # --- Topic Configuration ---
        self.sub_img = self.create_subscription(Image, '/cansat/camera_down/image_raw', self.image_callback, 10)
        self.pub_left = self.create_publisher(Wrench, '/cansat/left_arm_cmd', 10)
        self.pub_right = self.create_publisher(Wrench, '/cansat/right_arm_cmd', 10)
        self.pub_debug = self.create_publisher(Image, '/cansat/debug_image', 10)
        
        self.bridge = CvBridge()

        # PID Gain
        self.pid_x = {'kp': 2.0, 'ki': 0.00, 'kd': 0.0, 'integ': 0.0, 'prev': 0.0}
        self.pid_y = {'kp': 2.0, 'ki': 0.00, 'kd': 0.0, 'integ': 0.0, 'prev': 0.0}
        
        self.last_time = time.time()
        self.max_force = 2.0  
        self.base_brake = 0.6 
        
        # --- State Machine Parameters ---
        self.state = "IDLE"           # IDLE, ACTION, WAIT
        self.current_action = "NONE"  # LEFT, RIGHT, BRAKE, GLIDE

        self.action_start_time = 0.0
        self.wait_duration = 0.1      # 행동 후 안정화 대기 시간 (초)
        self.action_duration = 1.0    # 줄을 당기고 유지하는 시간 (초)
        
        # PID 계산값을 저장해둘 변수 (ACTION 상태에서 사용)
        self.stored_force_x = 0.0
        self.stored_force_y = 0.0

    def calculate_pid(self, error, pid_dict, dt):
        """PID 제어 계산 함수"""
        pid_dict['integ'] += error * dt
        pid_dict['integ'] = max(min(pid_dict['integ'], 5.0), -5.0)
        
        derivative = (error - pid_dict['prev']) / dt
        output = (pid_dict['kp'] * error) + (pid_dict['ki'] * pid_dict['integ']) + (pid_dict['kd'] * derivative)
        
        pid_dict['prev'] = error
        return output

    def image_callback(self, msg):
            #image processing
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w, _ = cv_image.shape
            cx, cy = w // 2, h // 2
            
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
                    
                    err_x_norm = (cx - tx) / (w / 2)
                    err_y_norm = (cy - ty) / (h / 2) 

          
            curr_time = time.time()
            dt = curr_time - self.last_time
            if dt <= 0: dt = 0.001
            
            raw_pid_x = self.calculate_pid(err_x_norm, self.pid_x, dt)
            raw_pid_y = self.calculate_pid(err_y_norm, self.pid_y, dt)
            self.last_time = curr_time

            left_val = 0.0
            right_val = 0.0


            # state switch
            if self.state == "ACTION":
                if curr_time - self.action_start_time <= self.action_duration:
                    
                    self.stored_force_x = min(abs(raw_pid_x), self.max_force)
                    self.stored_force_y = min(abs(raw_pid_y), self.max_force)
                    
                    force_x = self.stored_force_x
                    force_y = self.stored_force_y
                    
                    
                    
                    
                    if self.current_action == "LEFT":
                        left_val, right_val = force_x, 0.6
                    elif self.current_action == "RIGHT":
                        left_val, right_val = 0.6, force_x
                    elif self.current_action == "BRAKE":
                        left_val, right_val = force_y, force_y
                    elif self.current_action == "GLIDE":
                        left_val, right_val = 0.6, 0.6
                        
                    self.get_logger().info(f"{self.current_action}, {left_val} , {right_val}")
                else:
                    self.state = "WAIT"
                    self.action_start_time = curr_time 
                    self.get_logger().info("[ACTION DONE] Switching to WAIT (Holding Input)...")


            elif self.state == "WAIT":
                force_x = self.stored_force_x
                force_y = self.stored_force_y
                
                if self.current_action == "LEFT":
                    left_val, right_val = force_x, 0.0
                elif self.current_action == "RIGHT":
                    left_val, right_val = 0.0, force_x
                elif self.current_action == "BRAKE":
                    left_val, right_val = force_y, force_y
                elif self.current_action == "GLIDE":
                    left_val, right_val = 0.0, 0.0
                    
                if curr_time - self.action_start_time >= self.wait_duration:
                    self.state = "IDLE"
                    self.get_logger().info("[WAIT DONE] Ready for next command.")

            elif self.state == "IDLE":
                if not detected:
                    left_val, right_val = 0.0, 0.0
                else:
                    abs_x = abs(err_x_norm)
                    abs_y = abs(err_y_norm)
                    threshold = 0.15

                    if abs_x > threshold or abs_y > threshold:
                        self.state = "ACTION"
                        self.action_start_time = curr_time
                        
                        self.stored_force_x = min(abs(raw_pid_x), self.max_force)
                        self.stored_force_y = min(abs(raw_pid_y), self.max_force)
                        
                        if abs_x >= abs_y:
                            self.current_action = "LEFT" if err_x_norm > 0 else "RIGHT"
                        else:
                            self.current_action = "GLIDE" if err_y_norm > 0 else "BRAKE"
                                    
                        self.get_logger().info(f"[START ACTION] {self.current_action} (Wait for {self.wait_duration}s later)")

            self.publish_command(left_val, right_val)

            # --- Debug Visualization ---
            cv2.line(cv_image, (cx, 0), (cx, h), (255, 0, 0), 1)
            cv2.line(cv_image, (0, cy), (w, cy), (255, 0, 0), 1)
            
            if detected:
                cv2.circle(cv_image, target_pos, 8, (0, 255, 0), 2)
            
            status_color = (0, 255, 0) if self.state == "IDLE" else (0, 0, 255)
            remain_t = 0.0
            if self.state == "WAIT":
                remain_t = self.wait_duration - (curr_time - self.action_start_time)
            elif self.state == "ACTION":
                 remain_t = self.action_duration - (curr_time - self.action_start_time)
            
            state_text = f"State: {self.state} ({remain_t:.1f}s)"
            cv2.putText(cv_image, state_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            cv2.putText(cv_image, f"ErrX: {err_x_norm:.2f}  ErrY: {err_y_norm:.2f}", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(cv_image, f"L: {left_val:.1f}  R: {right_val:.1f}", (20, h-30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(cv_image, f"fx: {self.stored_force_x:.1f}  fy: {self.stored_force_y:.1f}", (20, h-60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

        except Exception as e:
            self.get_logger().error(f'Control Error: {e}')

    def publish_command(self, left, right):
        msg_l = Wrench()
        msg_l.force.x = float(left)
        
        msg_r = Wrench()
        msg_r.force.x = float(right)
        
        self.pub_left.publish(msg_l)
        self.pub_right.publish(msg_r)

def main(args=None):
    rclpy.init(args=args)
    node = CansatXYControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()