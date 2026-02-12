#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Wrench
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class CansatHomingControl(Node):
    def __init__(self):
        super().__init__('cansat_homing_control')
        
        self.sub_img = self.create_subscription(Image, '/cansat/camera_down/image_raw', self.image_callback, 10)
        self.pub_left = self.create_publisher(Wrench, '/cansat/left_arm_cmd', 10)
        self.pub_right = self.create_publisher(Wrench, '/cansat/right_arm_cmd', 10)
        self.pub_debug = self.create_publisher(Image, '/cansat/debug_image', 10)
        
        self.bridge = CvBridge()

        # --- 제어 파라미터 ---
        self.base_tension = 0.3  # 직진 시 기본 텐션 (안정성 확보)
        self.turn_force = 0.0    # 회전 시 당기는 힘 (강도)
        
        # [중요] Align & Glide 파라미터
        self.center_deadband = 0.55  # 화면 중앙 15% 이내면 직진 (불감대)
        self.control_interval = 0.5  # 제어 명령 주기 (너무 자주 바꾸면 흔들림)
        self.last_control_time = 0.0

        self.state = "SEARCH" # SEARCH, ALIGN_LEFT, ALIGN_RIGHT, GLIDE
        self.target_visible = False

    def image_callback(self, msg):
        try:
            # 1. 이미지 처리
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w, _ = cv_image.shape
            cx, cy = w // 2, h // 2
            
            # Red Detection
            blurred = cv2.GaussianBlur(cv_image, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, (0, 70, 50), (10, 255, 255)) | cv2.inRange(hsv, (170, 70, 50), (180, 255, 255))
            mask = cv2.erode(mask, None, iterations=2); mask = cv2.dilate(mask, None, iterations=2)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 2. 타겟 위치 분석
            err_x = 0.0
            self.target_visible = False
            target_pos = (cx, cy)

            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                M = cv2.moments(c)
                if M["m00"] > 0:
                    tx = int(M["m10"] / M["m00"])
                    ty = int(M["m01"] / M["m00"])
                    target_pos = (tx, ty)
                    self.target_visible = True
                    
                    # 정규화된 에러 (-1.0 ~ 1.0)
                    # (-)값: 타겟이 왼쪽 -> 왼쪽으로 회전 필요
                    # (+)값: 타겟이 오른쪽 -> 오른쪽으로 회전 필요
                    # (기존 코드와 부호 방향 일치시킴: 화면 좌표계 기준)
                    err_x = (tx - cx) / (w / 2) 

            # 3. 제어 로직 (0.5초마다 판단) [Align & Glide]
            curr_time = time.time()
            if curr_time - self.last_control_time > self.control_interval:
                self.last_control_time = curr_time
                self.decide_state(err_x)

            # 4. 모터 명령 생성 (State 기반)
            left_cmd, right_cmd = self.execute_state()
            self.publish_command(left_cmd, right_cmd)

            # 5. 디버깅
            self.draw_debug(cv_image, target_pos, err_x, left_cmd, right_cmd)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def decide_state(self, err_x):
        """ 에러를 보고 상태를 결정하는 함수 (가장 중요) """
        
        if not self.target_visible:
            self.state = "SEARCH" # 타겟 없으면 회전하며 찾기 (또는 직진)
            return

        # [Align Logic]
        # 타겟이 Deadband(중앙) 안에 들어오면 -> GLIDE (직진)
        if abs(err_x) < self.center_deadband:
            self.state = "GLIDE"
        
        # 타겟이 왼쪽(-Error)에 멀리 있음 -> 왼쪽으로 턴
        elif err_x < -self.center_deadband:
            self.state = "ALIGN_LEFT"
            
        # 타겟이 오른쪽(+Error)에 멀리 있음 -> 오른쪽으로 턴
        elif err_x > self.center_deadband:
            self.state = "ALIGN_RIGHT"

    def execute_state(self):
        """ 결정된 상태에 따라 힘을 반환 """
        l, r = 0.0, 0.0
        
        if self.state == "GLIDE":
            # [직진 모드] 양쪽을 동일하게 살짝 당겨서 활공 안정성 확보
            l = self.base_tension
            r = self.base_tension
            
        elif self.state == "ALIGN_LEFT":
            # [좌회전] 왼쪽을 당김 + 오른쪽은 풂 (확실한 턴)
            l = self.turn_force
            r = 0.0
            
        elif self.state == "ALIGN_RIGHT":
            # [우회전] 오른쪽을 당김 + 왼쪽은 풂
            l = 0.0
            r = self.turn_force
            
        elif self.state == "SEARCH":
            # 타겟을 찾을 때까지 완만한 선회 (또는 직진)
            l = self.base_tension + 0.2
            r = self.base_tension
            
        return l, r

    def publish_command(self, l, r):
        msg_l = Wrench(); msg_l.force.x = float(l)
        msg_r = Wrench(); msg_r.force.x = float(r)
        self.pub_left.publish(msg_l)
        self.pub_right.publish(msg_r)

    def draw_debug(self, img, pos, err, l, r):
        h, w, _ = img.shape
        cx = w // 2
        
        # Deadband 영역 표시 (이 안에 들어오면 직진함)
        db_w = int(w * self.center_deadband)
        cv2.line(img, (cx - db_w, 0), (cx - db_w, h), (0, 255, 255), 1) # 노란선
        cv2.line(img, (cx + db_w, 0), (cx + db_w, h), (0, 255, 255), 1)

        if self.target_visible:
            cv2.circle(img, pos, 10, (0, 255, 0), 3)
            cv2.line(img, (cx, h//2), pos, (0, 255, 0), 2)
            
        # 텍스트 정보
        state_color = (0, 255, 0) if self.state == "GLIDE" else (0, 0, 255)
        cv2.putText(img, f"State: {self.state}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, state_color, 2)
        cv2.putText(img, f"ErrX: {err:.2f}", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(img, f"L: {l:.1f}  R: {r:.1f}", (20, h-30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

def main():
    rclpy.init()
    node = CansatHomingControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()