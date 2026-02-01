import os
import time
import math
import subprocess
import re

# ==========================================
# 1. PID 클래스 (사용자가 제공한 코드 참고)
# ==========================================
class AltitudePID:
    def __init__(self, target_altitude_, kp, ki, kd):
        self.target = target_altitude_
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.prev_time = time.time()
        
        # 호버링을 유지하기 위한 기본 모터 속도 (중력 보상)
        # 드론 무게에 따라 다르지만 보통 500~700 사이
        self.base_rpm = 800.0 

    def compute(self, current_altitude):
        current_time = time.time()
        dt = current_time - self.prev_time
        
        if dt <= 0: return 0
        
        # 오차 계산 (목표 - 현재)
        error = self.target - current_altitude
        
        # 적분 (I)
        self.integral_error += error * dt
        # 적분 누적 제한 (Anti-windup)
        self.integral_error = max(-500, min(500, self.integral_error))

        # 미분 (D)
        derivative = (error - self.prev_error) / dt
        
        # PID 출력 계산
        output = (self.kp * error) + (self.ki * self.integral_error) + (self.kd * derivative)
        
        self.prev_error = error
        self.prev_time = current_time
        
        return output

# ==========================================
# 2. 센서 데이터 가져오기 (CLI 파싱 트릭)
# ==========================================
def get_current_altitude():
    """
    Topic: /model/cansat/pose
    """
    try:
        # ign topic -e (echo) -n 1 (count) -t (topic)
        cmd = ["ign", "topic", "-e", "-n", "1", "-t", "/gui/camera/pose"]

        print(f"DEBUG: Executing command -> {' '.join(cmd)}")
        result = subprocess.check_output(cmd, timeout=1.0, stderr=subprocess.STDOUT).decode('utf-8')
        
        # print(f"DEBUG: Raw result -> {result[:50]}...")

        # parsing
        match = re.search(r'position\s*\{[^}]*z:\s*([\d\.-]+)', result, re.DOTALL) # Z information (Height)
        
        if match:
            z_val = float(match.group(1))
            # print(f"DEBUG: Parsed Height: {z_val}")
            return z_val
        else:
            print("WARNING: 'position z' not found in output.")
            return None
            
    except subprocess.TimeoutExpired:
        print("ERROR: Timeout! Topic data is not coming in (0.5s).")
        return None
    except subprocess.CalledProcessError as e:
        print(f"ERROR: Command failed with return code {e.returncode}")
        print(f"Output: {e.output.decode('utf-8') if e.output else 'No output'}")
        return None
    except Exception as e:
        print(f"ERROR: Unexpected error: {e}")
        return None
    
    
    
# ==========================================
# 3. 메인 실행 루프
# ==========================================
def main():
    # 목표 고도 2.0m 설정
    # Kp, Ki, Kd 값은 시뮬레이션 환경에 맞춰 튜닝 필요
    
    target_altitude = 15.0
    pid = AltitudePID(target_altitude_=target_altitude, kp=15.0, ki=0.0, kd=0.0)
    
    print("\n=== Quadcopter Hovering Control Start ===")
    print(f"Target Altitude: {target_altitude}")
    
    try:
        while True:
            
            current_alt = get_current_altitude()
            if current_alt is None:
                continue
            
            # 2. PID 계산
            control_output = pid.compute(current_alt)
            
            # 3. 최종 모터 속도 계산 (기본 속도 + PID 보정값)
            final_rpm = pid.base_rpm + control_output
            
            # 모터 속도는 0보다 작을 수 없고, 최대값(예: 1500)을 넘을 수 없음
            final_rpm = max(0, min(1500, final_rpm))
            
            # 4. 명령 전송
            # 소수점 제거 (int)
            rpm_int = int(final_rpm)
            cmd_vel = f"ign topic -t /X3/gazebo/command/motor_speed --msgtype ignition.msgs.Actuators -p 'velocity:[{rpm_int}, {rpm_int}, {rpm_int}, {rpm_int}]'"
            
            os.system(cmd_vel)
            
            # 상태 출력
            print(f"Alt: {current_alt:.2f}m | Target: {pid.target}m | Motor RPM: {rpm_int} | PID Out: {control_output:.1f}")
            
            # CLI 실행 속도 때문에 약간의 딜레이가 자연스럽게 생깁니다.
            # 너무 빠르면 시스템 부하가 걸리므로 time.sleep 조절
            time.sleep(0.05) 

    except KeyboardInterrupt:
        print("\nLanding...")
        # 모터 정지 명령
        os.system("ign topic -t /X3/gazebo/command/motor_speed --msgtype ignition.msgs.Actuators -p 'velocity:[0, 0, 0, 0]'")
        print("Disarmed.")

if __name__ == "__main__":
    main()