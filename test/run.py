import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class ParafoilCansat:
    def __init__(self):
        # 1. 물리 상수 설정
        self.g = 9.81  # 중력 가속도 (m/s^2)
        self.m = 2.0   # Cansat + Parafoil 총 질량 (kg)
        
        # 2. 관성 모멘트 (Inertia Tensor) - 근사치
        # Ixx, Iyy, Izz, Ixy, Ixz, Iyz
        self.I = np.diag([0.1, 0.1, 0.2]) 
        
        # 3. 질량 행렬 (Rigid Body Mass Matrix, 6x6)
        # 선속도(v)와 각속도(w) 간의 커플링이 없다고 가정 (무게중심=원점)
        self.M_rb = np.zeros((6, 6))
        self.M_rb[:3, :3] = np.eye(3) * self.m
        self.M_rb[3:, 3:] = self.I
        
        # 4. 부가 질량 행렬 (Added Mass Matrix) - 매우 중요!
        # 패러포일은 공기를 가두므로 실제 질량보다 무겁게 움직임 (대각 성분만 있다고 단순화)
        self.M_add = np.diag([0.5, 0.2, 1.5, 0.1, 0.1, 0.1]) 
        
        # 최종 질량 행렬 M (시스템의 관성)
        self.M_total = self.M_rb + self.M_add
        self.M_inv = np.linalg.inv(self.M_total)

    def rotation_matrix(self, phi, theta, psi):
        """오일러 각(NED) -> 회전 행렬 R_BI 계산"""
        c_phi, s_phi = np.cos(phi), np.sin(phi)
        c_th, s_th = np.cos(theta), np.sin(theta)
        c_psi, s_psi = np.cos(psi), np.sin(psi)

        R = np.array([
            [c_th*c_psi, c_th*s_psi, -s_th],
            [s_phi*s_th*c_psi - c_phi*s_psi, s_phi*s_th*s_psi + c_phi*c_psi, s_phi*c_th],
            [c_phi*s_th*c_psi + s_phi*s_psi, c_phi*s_th*s_psi - s_phi*c_psi, c_phi*c_th]
        ])
        return R

    def dynamics(self, t, state, u_control):
        """
        state: [x, y, z, phi, theta, psi, u, v, w, p, q, r] (12x1)
        u_control: [delta_left, delta_right] (제어 입력)
        """
        # 상태 변수 분해
        pos = state[0:3]    # 위치 (x, y, z)
        att = state[3:6]    # 자세 (phi, theta, psi)
        nu = state[6:12]    # 바디 속도 (u, v, w, p, q, r)
        
        vel_b = nu[0:3]     # 선속도
        omega_b = nu[3:6]   # 각속도
        
        phi, theta, psi = att

        # --- A. 운동학 (Kinematics) ---
        # 1. 속도 변환 (Body -> Inertial)
        R_bi = self.rotation_matrix(phi, theta, psi)
        vel_i = R_bi.T @ vel_b  # NED 좌표계 속도

        # 2. 각속도 변환 (Body Rates -> Euler Rates)
        # 특이점(Singularity) 방지를 위해 theta가 90도 근처면 주의 필요
        tt = np.tan(theta)
        ct = np.cos(theta)
        T_matrix = np.array([
            [1, np.sin(phi)*tt, np.cos(phi)*tt],
            [0, np.cos(phi), -np.sin(phi)],
            [0, np.sin(phi)/ct, np.cos(phi)/ct]
        ])
        att_dot = T_matrix @ omega_b

        # --- B. 동역학 (Dynamics) ---
        # 식: M * nu_dot + C * nu + D * nu + g = tau
        # 변형: nu_dot = M_inv * (tau - C*nu - D*nu - g)

        # 1. 중력 (g_vec) - Inertial Frame의 중력을 Body Frame으로 변환
        F_gravity_inertial = np.array([0, 0, self.m * self.g])
        F_gravity_body = R_bi @ F_gravity_inertial
        # 중력 벡터 (6x1): 힘은 있고 모멘트는 0 (CG 기준)
        g_vec = np.zeros(6)
        g_vec[:3] = -F_gravity_body # 우변으로 넘길 때 부호 주의 (여기선 복원력 항으로 정의하여 좌변에 있다면)
        # 보통 모델링에서 중력을 외력(tau)에 포함시키기도 하고, 복원력(g) 항으로 빼기도 합니다.
        # 여기서는 편의상 "외력(External Force)" 취급하여 tau 쪽에 더하겠습니다.

        # 2. 코리올리 힘 (C * nu) - 근사식: Cross Product 이용
        # 강체 부분만 고려한 단순화된 코리올리 힘
        F_cor = np.cross(omega_b, self.m * vel_b)
        M_cor = np.cross(omega_b, self.I @ omega_b)
        C_nu = np.concatenate([F_cor, M_cor])

        # 3. 공기역학적 감쇠 (D * nu) - 단순화된 선형+이차 모델
        # 실제로는 받음각(alpha), 옆미끄러짐각(beta)에 따른 CL, CD 테이블 조회 필요
        # 여기서는 속도에 비례하는 저항(Damping)만 있다고 가정
        D_matrix = np.diag([0.5, 0.8, 2.0, 0.1, 0.1, 0.1]) # 임의의 감쇠 계수
        D_nu = D_matrix @ nu 

        # 4. 제어 입력 및 외력 (tau)
        # 패러포일 조향: 왼쪽/오른쪽 브레이크 당김
        d_L, d_R = u_control
        
        # 입력에 따른 힘/모멘트 생성 (간단한 매핑 모델)
        # 예: 한쪽을 당기면 Yaw 모멘트 발생, 둘 다 당기면 Drag(x축 힘) 증가
        F_control_x = -5.0 * (d_L + d_R) # 저항 증가 (Braking)
        M_control_z = 2.0 * (d_R - d_L)  # Yaw 모멘트 (Turning)
        
        tau = np.zeros(6)
        tau[0] = F_control_x
        tau[5] = M_control_z
        
        # 중력을 외력으로 포함
        tau[:3] += F_gravity_body

        # 5. 가속도 계산 (Main Equation)
        # nu_dot = M_inv * (tau - C_nu - D_nu)
        rhs = tau - C_nu - D_nu
        nu_dot = self.M_inv @ rhs

        # 결과 반환: [pos_dot, att_dot, nu_dot]
        return np.concatenate([vel_i, att_dot, nu_dot])

# --- 시뮬레이션 실행 ---

def run_simulation():
    model = ParafoilCansat()
    
    # 초기 상태 설정
    # 위치(0,0,-1000m 상공), 자세(0,0,0), 속도(전진 15m/s)
    y0 = np.array([0, 0, -1000, 0, 0, 0, 15, 0, 2, 0, 0, 0])
    
    # 시간 설정 (0초 ~ 30초)
    t_span = (0, 30)
    t_eval = np.linspace(0, 30, 300)
    
    # 제어 입력 (오른쪽으로 선회: R=0.2, L=0.0)
    u_input = [0.0, 0.2] 

    # ODE Solver 실행
    sol = solve_ivp(
        fun=lambda t, y: model.dynamics(t, y, u_input),
        t_span=t_span,
        y0=y0,
        t_eval=t_eval,
        method='RK45'
    )

    return sol

# --- 결과 시각화 ---
sol = run_simulation()
x, y, z = sol.y[0], sol.y[1], sol.y[2]

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.plot(x, y, -z) # NED 좌표계이므로 z를 뒤집어서 고도로 표현
ax.set_title("Parafoil CANSAT Trajectory (Right Turn)")
ax.set_xlabel("North (m)")
ax.set_ylabel("East (m)")
ax.set_zlabel("Altitude (m)")
plt.show()
