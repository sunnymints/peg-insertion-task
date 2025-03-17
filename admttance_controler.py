
import numpy as np



    
    
def solution(fext, M, B, K,t=1/60):

    xd = (B * fext / (2 * K * np.sqrt(B ** 2 - 4 * K * M)) - fext / (2 * K)) * np.exp(
        t * (-B - np.sqrt(B ** 2 - 4 * K * M)) / (2 * M)) + (
                    -B * fext / (2 * K * np.sqrt(B ** 2 - 4 * K * M)) - fext / (2 * K)) * np.exp(
        t * (-B + np.sqrt(B ** 2 - 4 * K * M)) / (2 * M)) + fext / K

    return complex(xd).real

def Dynamic_rotation_center(FT):

    K=(1 - insert_depth / 0.03)
    # 计算当前旋转中心在eelink下的相对深度
    if insert_depth > 0:
        eelink_relative_length = 0.0925 - insert_depth / 2
    else:
        eelink_relative_length = 0.0925
        
    Dynamic_center_torqueX = FT[3]
    Dynamic_center_torqueY = FT[4] + FT[2] * eelink_relative_length
    Dynamic_center_torqueZ = FT[5] - FT[1] * eelink_relative_length

    Dynamic_center_forces = [FT[0]*K, FT[1]*K, FT[2]*K, Dynamic_center_torqueX*K, Dynamic_center_torqueY*K,
                                Dynamic_center_torqueZ*K, eelink_relative_length]
    del FT

    return Dynamic_center_forces
# 位置导纳控制函数
def admittance_position(M_position, B_position, K_position,FT):
    # posture_new = p.getLinkState(self.ur5_id, 7)[5]
    m_position = []

    fext = Dynamic_rotation_center(FT)[0:3]  # 动态力矩中心

    if np.abs(np.array(fext).max()) < 10:
        K = 3
    else:
        K=1

    for i in range(3):
        xd = solution(-fext[i]*K, M_position, B_position[i], K_position)
        m_position.append(xd)

    m_position = [m_position[0], m_position[1], m_position[2]]
    return m_position

# 姿态导纳控制函数
def admittance_posture( M_posture, B_posture, K_posture,FT):
    posture = []
    
    f_t = Dynamic_rotation_center(FT)[0:6]  # 动态力矩中心
    print('旋转中心力：',f_t)
    fext = f_t[:3]
    torue = f_t[3:]

    if np.abs(np.array(f_t)).max() < 0.5:
        posture=[0,0,0]
    else:
        for i in range(3):
            xd = solution(-torue[i]*10, M_posture, B_posture, K_posture)
            posture.append(xd)
    posture_x = 0  # posture[0]
    posture_y = posture[1]
    posture_z = posture[2]
    m_posture = [posture_x, posture_y, posture_z]
    return m_posture

if __name__ == '__main__':

    # 参数赋值
    M_posture = 0.01
    B_posture = 8
    K_posture = 50  #原始

    M_position = 0.1
    B_position =np.zeros(3)
    B_position[0] =2000
    B_position[1] = 900
    B_position[2] = 900
    K_position=800
    FT = [0.4, 2.0, -0.9, 0.0, 1.05, 0.14]
    insert_depth = 0.000

    m_position = admittance_position(M_position, B_position, K_position,FT)
    m_posture = admittance_posture(M_posture, B_posture, K_posture,FT)

    print('****************')