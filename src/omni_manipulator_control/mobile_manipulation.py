#!usr/bin/env pyhon3

from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetLinkState
import modern_robotics as mr
import matplotlib.pyplot as plt
import numpy as np
import rospy
import math
import time

I_term = np.zeros(6)

## robot_state  [phi, x, y, u1, u2, u3, u4, th1, th2, th3, th4, th5, th6] ##
phi, x_pos, y_pos = 0.0, 0.0, 0.0
w1, w2, w3, w4 = 0.0, 0.0, 0.0, 0.0    
rot_x_init, rot_y_init, rot_z_init = 0.0, 0.0, 0.0
thetalist = np.array([0.001, 0.001, 0.001, 0.001, 0.001, 0.001])
robot_state_init = np.r_[np.array([phi, x_pos, y_pos, w1, w2, w3, w4]), thetalist]
vel = np.zeros(3)

# Mecanum mobile platform #
r = 0.0762 # radius of wheel
l = 0.23 #length between {b} and wheels
w = 0.25225 #depth between {b} and wheel
alpha = l + w
F_wheel = r/4 * np.array([[-1/alpha, 1/alpha, 1/alpha, -1/alpha], [1, 1, 1, 1], [-1, 1, -1, 1]])
F_wheel_6 = np.r_[[np.array([0, 0, 0, 0])], [np.array([0, 0, 0, 0])], F_wheel, [np.array([0, 0, 0, 0])]]

## Manipulator ##
#Link length : unit [m]
l = np.array([0.085, 0.198, 0.196, 0.116, 0.116, 0.116])

#M matrix
M = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,l[0]+l[1]+l[2]+l[4]],[0,0,0,1]])

Blist = np.array([[0, 0, 1, 0, 0, 0],
                [0, 1, 0, l[1]+l[2]+l[4], 0, 0],
                [0, 1, 0, l[2]+l[4], 0, 0],
                [0, 1, 0, l[4], 0, 0],
                [0, 0, 1, l[5], 0, 0],
                [0, 1, 0, 0, 0, 0]]).T

T_b0 = np.array([[1, 0, 0, 0.17],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]])

T_sb = np.array([[math.cos(phi), -math.sin(phi), 0, x_pos],
            [math.sin(phi), math.cos(phi), 0, y_pos],
            [0, 0, 1, 0.19458],
            [0, 0, 0, 1]])
T67 = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.128],[0,0,0,1]])

# End-effector 초기위치 출력
T_0e = mr.FKinBody(M, Blist, thetalist)
T_0e = np.dot(T_0e,T67)
Tse_i = np.dot(np.dot(T_sb, T_b0), T_0e)
Tse_i = np.round(Tse_i,4)

x_init = Tse_i[0][3]
y_init = Tse_i[1][3]
z_init = Tse_i[2][3]

joint_1 = rospy.Publisher('/omni_manipulator/joint_1_pos/command', Float64, queue_size=10)
joint_2 = rospy.Publisher('/omni_manipulator/joint_2_pos/command', Float64, queue_size=10)
joint_3 = rospy.Publisher('/omni_manipulator/joint_3_pos/command', Float64, queue_size=10)
joint_4 = rospy.Publisher('/omni_manipulator/joint_4_pos/command', Float64, queue_size=10)
joint_5 = rospy.Publisher('/omni_manipulator/joint_5_pos/command', Float64, queue_size=10)
joint_6 = rospy.Publisher('/omni_manipulator/joint_6_pos/command', Float64, queue_size=10)
gripper_R_pub = rospy.Publisher('/omni_manipulator/gripper/command', Float64, queue_size=10)
gripper_L_pub = rospy.Publisher('/omni_manipulator/gripper_sub/command', Float64, queue_size=10)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
rospy.init_node('omni_manipulator', anonymous=True)
rate = rospy.Rate(100) # 100hz

def TrajectoryGenerator(Tse_i,Tsc_i,Tsc_f,Tce_g,Tce_s):

    """
    Tse_i: End-effector's initial homogeneous transformation.
    Tsc_i: Fist target tennis ball homogeneous transformation.
    Tsc_f: Second target tennis ball homogeneous transformation.
    Tce_g: End-effector's homogeneous transformation when it is grasping tennis ball
    Tce_s: End-effector's homogeneous transformation before it is grasping tennis ball
   
    """

    Tse_gi = np.dot(Tsc_i,Tce_g) # The end-effector's configuration relative to the world when it is grasping the initial cube
    Tse_gf = np.dot(Tsc_f,Tce_g) # The end-effector's configuration relative to the world when it is grasping the final cube

    traj1_new = mr.CartesianTrajectory(Tse_i,Tse_gi,12,1200,5) # Move to first target pos
    traj2_new = mr.CartesianTrajectory(Tse_gi,Tse_gf,6,600,5) # Move to second target pos

    
    return  traj1_new, traj2_new

def assum_zero(val):
    """
    Convert to 0, if less than 0.001
    """
   
    return abs(val) < 1e-4

def Roll(ang_r):
    """
    Rotate X-axis

    ang_r : X angle

    """
    return np.array([[1,0,0],[0,np.cos(ang_r),-np.sin(ang_r)],[0,np.sin(ang_r),np.cos(ang_r)]])

def Pitch(ang_p): 
    """
    Rotate Y-axis

    ang_p : Y angle

    """
    return np.array([[np.cos(ang_p),0,np.sin(ang_p)],[0,1,0],[-np.sin(ang_p),0,np.cos(ang_p)]])

def Yaw(ang_y):
    """
    Rotate Z-axis

    ang_y : Z angle

    """
    return np.array([[np.cos(ang_y),-np.sin(ang_y),0],[np.sin(ang_y),np.cos(ang_y),0],[0,0,1]])

def get_X_d(x, y, z, roll, pitch, yaw):
    """
    Get homogeneous transformation

    x : x position
    y : y position
    z : z position

    roll : Rotate X-axis
    pitch : Rotate y-axis
    yaw :  Rotate Z-axis

    """

    R = np.dot(np.dot(Roll(roll), Pitch(pitch)), Yaw(yaw))
    xyz = np.array([x, y, z])
    
    T_d = np.vstack([np.c_[R, xyz], np.array([0,0,0,1])])

    return T_d

def joint_angle_limit(joint_angle):
    """
    6-DoF manipulator joint angle limit

    joint_angle : np.array([theta1, theta2, theta3, theta4, theta5, theta6])

    """

    for i in range(len(joint_angle)):
        if i == 0:
            if joint_angle[i] > np.pi:
                joint_angle[i] = np.pi
            if joint_angle[i] < -np.pi:
                joint_angle[i] = -np.pi
        elif i == 1 :
            if joint_angle[i] > 70* np.pi/180:
                joint_angle[i] = 70* np.pi/180
            if joint_angle[i] < -70* np.pi/180:
                joint_angle[i] = -70* np.pi/180
        elif i > 1:
            if joint_angle[i] > np.pi/2:
                joint_angle[i] = np.pi/2
            if joint_angle[i] < -np.pi/2:
                joint_angle[i] = -np.pi/2

    return joint_angle


def convert_radian_from_npi_to_pi(thetalist):

    """
    A function that allows the joint angle to range from -pi to np

    thetalist : 6DoF manipulator's joint

    """
    
    if type(thetalist) is not np.ndarray:
        if(-3.14 > thetalist):
            thetalist = thetalist + np.pi*2
        elif(thetalist > 3.14):
            thetalist = thetalist - np.pi*2
    else:
        for i in range(len(thetalist)):
            while(-3.14 > thetalist[i]):
                thetalist[i] = thetalist[i] + np.pi*2
            while(thetalist[i] > 3.14):
                thetalist[i] = thetalist[i] - np.pi*2
    
    return thetalist

def Trapezoidal_Traj_Gen_Given_Amax_and_T(amax,T,dt):
    """
    
    Create a trapezoidal velocity profile

    amax : Max acceleration
    T : Time
    dt : Amount of time change
    
    """
    a = amax
    
    if a*math.pow(T,2) >= 4:     
        v = 0.5*(a*T - math.pow(a,0.5)*math.pow((a*math.pow(T,2)-4),0.5))
    else:
        return False, False            
    
    time = 0
    t_save = time
    traj_save = np.array([0,0,0])
    while T >= time:
        if time >= 0 and time <= (v/a):
            sddot = a
            sdot = a*time
            s = 0.5*a*math.pow(time,2)
            
        if time > (v/a) and time <= (T-v/a):
            sddot = 0
            sdot = v 
            s = v*time - 0.5*math.pow(v,2)/a
            
        if time > (T-v/a) and time <= T:
            sddot = -a
            sdot = a*(T-time)
            s = (2*a*v*T - 2*math.pow(v,2) - math.pow(a,2)*math.pow((time-T),2))/(2*a)
        
        t_save = np.vstack((t_save, time))
        traj_save = np.vstack((traj_save, np.array([s,sdot,sddot])))
        time += dt

    return t_save, traj_save

def Path_Gen(start,goal,traj):
    path = start + traj*(goal-start)
    return path

def get_Jacobian(robot_state):
    """
    phi : Chassis angle
    x_pos : Chassis x position
    y_pos : Chassis y position
    theta : Joint angle
    T_sb : Homogeneous transformation of fixed space frame {s} to chassis frame {b}
    T_b0 : Homogeneous transformation of chassis frame {b} to manipulator's space frmae {0}
    T_0e : Homogeneous transformation of manipulator's space frmae {0} to end-effector's frmae {e}
    T67 : Homogeneous transformation of link_6 frame to end-effector's frmae {e}
    X : Homogeneous transformation of fixed space frame {s} to end-effector's frmae {e}
    J_arm : Manipulator body jacobian
    J_base : Chassis jacobian
    J_e : [J_arm J_base]
    
    """
    phi = robot_state[0]
    x_pos = robot_state[1]
    y_pos = robot_state[2]
    theta = robot_state[7:]
    l = np.array([0.085, 0.198, 0.196, 0.116, 0.116, 0.116])

    T_sb = np.array([[math.cos(phi), -math.sin(phi), 0, x_pos],
                [math.sin(phi), math.cos(phi), 0, y_pos],
                [0, 0, 1, 0.19458],
                [0, 0, 0, 1]])
    
    T_0e = mr.FKinBody(M, Blist, theta)
    T_0e = np.dot(T_0e,T67)
    X = np.dot(np.dot(T_sb, T_b0), T_0e)

    J_arm = mr.JacobianBody(Blist, theta)
    J_base = np.dot(mr.Adjoint(np.dot(mr.TransInv(T_0e), mr.TransInv(T_b0))), F_wheel_6)
    J_e = np.c_[J_base, J_arm]

    return X, J_e

def PI_control(X_d, X_d_next, X, Kp, Ki, dt, J_e):
    global I_term
    """
    X_d : n th homogeneous transformation
    X_d_next : n+1 th homogeneous transformation
    V_d : Desired twist
    dt : Time step
    V_cal : End-effector twist : V_d + Kp*Xe + Ki*int(Xe)*dt
    u_th_d : Wheel & joint speed
    
    """

    p_gain = Kp * np.identity(6)
    i_gain = Ki * np.identity(6)
    Vd = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(X_d),X_d_next))/dt)
    X_err = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(X),X_d)))
    P_term = np.dot(p_gain, X_err)
    I_term +=  X_err * dt
    V_cal = np.dot(mr.Adjoint(np.dot(mr.TransInv(X), X_d)),Vd) + P_term + np.dot(i_gain, I_term)
    u_th_d = np.dot(np.linalg.pinv(J_e), V_cal)

    return u_th_d, X_err


def Next_step(u_th_d, robot_state, dt):
    """
    theta_dot : Joint speed
    u : Wheel velocity
    delta_wheel_ang : delta_theta (Odometry update 때 사용)
    T_sb : Transformation matrix chassis frame {b} to fixed space frame {s}
    delta_qb : delta qb(phi, x, y)
    chassis_vel : Velocity of chassis (Twist Vb)
    
    """
    # max_speed_wheel = 6.32*14 #[rad/s => 로봇속도 : 최대 1m/s] 
    # max_speed_joint = 500 * np.pi/180

    # for k in range(len(u_th_d)):
    #     if k < 4:
    #         if u_th_d[k] > max_speed_wheel:
    #             u_th_d[k] = max_speed_wheel
    #         if u_th_d[k] < -max_speed_wheel:
    #             u_th_d[k] = -max_speed_wheel
    #     if k > 4:
    #         if u_th_d[k] > max_speed_joint:
    #             u_th_d[k] = max_speed_joint
    #         if u_th_d[k] < -max_speed_joint:
    #             u_th_d[k] = -max_speed_joint

    phi = robot_state[0]
    x_pos = robot_state[1]
    y_pos = robot_state[2]
    wheel_ang = robot_state[3:7]
    thetalist = robot_state[7:]

    theta_dot = u_th_d[4:]
    u = u_th_d[:4]
    delta_wheel_ang = u*dt
    
    # Find delta_qb
    T_sb = np.array([[1.0, 0.0, 0.0],[0, math.cos(phi), -math.sin(phi)], [0, math.sin(phi), math.cos(phi)]])
    Vb = np.dot(F_wheel,delta_wheel_ang)
    wbz, vbx, vby = Vb
    
    if assum_zero(wbz):
        d_qb = np.array([0, vbx, vby])

    else:
        d_qb = np.array([wbz, (vbx * math.sin(wbz) + vby * (math.cos(wbz) - 1))/wbz, (vby * math.sin(wbz) + vbx * (1 - math.cos(wbz)))/wbz])

    # Update chassis state (phi, x, y)
    delta_q = np.dot(T_sb, d_qb)
    phi_update = phi + delta_q[0]
    x_pos_update = x_pos + delta_q[1]
    y_pos_update = y_pos + delta_q[2]
    
    # Update joint anlge
    new_thetalist = thetalist + theta_dot * dt
    for i in range(len(new_thetalist)):
        new_thetalist[i] = convert_radian_from_npi_to_pi(new_thetalist[i])
    
    new_thetalist = joint_angle_limit(new_thetalist)

    # Update wheel angle
    new_wheel_ang = wheel_ang + delta_wheel_ang

    update_state = np.r_[phi_update, x_pos_update, y_pos_update, new_wheel_ang, new_thetalist]
    chassis_vel = np.dot(F_wheel,u) # Gazebo 에서 로봇을 움직이기 위해 사용 rostopic으로 /cmd_vel에 쏴줌

    return update_state, chassis_vel

def get_endeffector():
    """
    A function indicating the position and orientation of the end-effector in Gazebo.
    """
    end_effector_info_prox = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    # rospy.wait_for_service('/gazebo/get_link_state')
    info = end_effector_info_prox( 'end_effector' , '' )

    x = info.link_state.pose.position.x
    y = info.link_state.pose.position.y
    z = info.link_state.pose.position.z
    q_x = info.link_state.pose.orientation.x
    q_y = info.link_state.pose.orientation.y
    q_z = info.link_state.pose.orientation.z
    q_w = info.link_state.pose.orientation.w
    
    # Convert quaternion to euler
    t0 = +2.0 * (q_w * q_x + q_y * q_z)
    t1 = +1.0 - 2.0 * (q_x * q_x + q_y * q_y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (q_w * q_y - q_z * q_x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (q_w * q_z + q_x * q_y)
    t4 = +1.0 - 2.0 * (q_y * q_y + q_z * q_z)
    yaw_z = math.atan2(t3, t4)

    return x, y, z, roll_x, pitch_y, yaw_z # in radians

def rotationMatrixToEulerAngles(R) :
    """
    A function of finding the rotation angle at Td obtained from path generation
    Using to get Rot_x,y,z of the end-effector.
    
    R : 3 X 3 rotation matrix

    """
    # assert(isRotationMatrix(R))
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])

def check_chassis_vel_zero(chassis_vel):
    n = len(chassis_vel)
    for i in range(n):
        if chassis_vel[i] < 0:
            if chassis_vel[i] < 1e-1:
                chassis_vel[i] = 0.
        if chassis_vel[i] > 0:
            if abs(chassis_vel[i]) < 1e-1:
                chassis_vel[i] = 0.
    return chassis_vel

def Gripper_open(gripper_state):
    if gripper_state == 1:
        print("Gripper is already opened")
    
    elif gripper_state == 0:
        t_end = time.time() + 2.8

        while(time.time() < t_end):
            # send_cmd('J6', 'rcservo', 37500)
            gripper_L_pub.publish(np.float64(0.0))
            gripper_R_pub.publish(np.float64(0.0))

        # send_cmd('J6', 'rcservo', 35000)
        gripper_state = 1
        print("Gripper open")

    else:
        print("Wrong gripper state")

    return gripper_state
    
def Gripper_close(gripper_state):
    if gripper_state == 0:
        print("Gripper is already closed")
    
    elif gripper_state == 1:
        t_end = time.time() + 2.8

        while(time.time() < t_end):
            # send_cmd('J6', 'rcservo', 30500)
            gripper_L_pub.publish(np.float64(0.15))
            gripper_R_pub.publish(np.float64(0.15))

        # send_cmd('J6', 'rcservo', 35000)
        gripper_state = 0
        print("Gripper close")

    else:
        print("Wrong griper state")

    return gripper_state 

def Put_in_the_basket(current_joint_angle, gripper_state, chassis_data, last_motion):

    global ecd_data
    
    DEG2RAD = lambda deg: deg*np.pi/180
    RAD2DEG = lambda rad: rad*180/np.pi

    target_joint = np.array([0., 0., -15*np.pi/180, -np.pi/2., np.pi/2, 0.])
    
    if not last_motion :
        
        t, traj = Trapezoidal_Traj_Gen_Given_Amax_and_T(1.5, 6, 0.01)
        th1_traj_1 = Path_Gen(current_joint_angle[0], target_joint[0], traj[:,0])
        th2_traj_1 = Path_Gen(current_joint_angle[1], target_joint[1], traj[:,0])
        th3_traj_1 = Path_Gen(current_joint_angle[2], target_joint[2], traj[:,0])
        th4_traj_1 = Path_Gen(current_joint_angle[3], target_joint[3], traj[:,0])
        th5_traj_1 = Path_Gen(current_joint_angle[4], target_joint[4], traj[:,0])
        th6_traj_1 = Path_Gen(current_joint_angle[5], target_joint[5], traj[:,0])

        th1_traj_2 = Path_Gen(target_joint[0], current_joint_angle[0], traj[:,0])
        th2_traj_2 = Path_Gen(target_joint[1], current_joint_angle[1], traj[:,0])
        th3_traj_2 = Path_Gen(target_joint[2], current_joint_angle[2], traj[:,0])
        th4_traj_2 = Path_Gen(target_joint[3], current_joint_angle[3], traj[:,0])
        th5_traj_2 = Path_Gen(target_joint[4], current_joint_angle[4], traj[:,0])
        th6_traj_2 = Path_Gen(target_joint[5], current_joint_angle[5], traj[:,0])
    else :
        
        t, traj = Trapezoidal_Traj_Gen_Given_Amax_and_T(1.5, 6, 0.01)
        th1_traj_1 = Path_Gen(current_joint_angle[0], target_joint[0], traj[:,0])
        th2_traj_1 = Path_Gen(current_joint_angle[1], target_joint[1], traj[:,0])
        th3_traj_1 = Path_Gen(current_joint_angle[2], target_joint[2], traj[:,0])
        th4_traj_1 = Path_Gen(current_joint_angle[3], target_joint[3], traj[:,0])
        th5_traj_1 = Path_Gen(current_joint_angle[4], target_joint[4], traj[:,0])
        th6_traj_1 = Path_Gen(current_joint_angle[5], target_joint[5], traj[:,0])

        th1_traj_2 = Path_Gen(target_joint[0], 0., traj[:,0])
        th2_traj_2 = Path_Gen(target_joint[1], 0., traj[:,0])
        th3_traj_2 = Path_Gen(target_joint[2], 0., traj[:,0])
        th4_traj_2 = Path_Gen(target_joint[3], 0., traj[:,0])
        th5_traj_2 = Path_Gen(target_joint[4], 0., traj[:,0])
        th6_traj_2 = Path_Gen(target_joint[5], 0., traj[:,0])


    for k in range(2):
        print("Put in the basket")

        if k == 1:
            gripper_state = Gripper_open(gripper_state)
            time.sleep(1.6)

        for i in range(len(th1_traj_1)):
            if k == 0:
                theta = np.array([th1_traj_1[i], th2_traj_1[i], th3_traj_1[i], th4_traj_1[i], th5_traj_1[i], th6_traj_1[i]])
            elif k == 1:
                theta = np.array([th1_traj_2[i], th2_traj_2[i], th3_traj_2[i], th4_traj_2[i], th5_traj_2[i], th6_traj_2[i]])

            joint_1.publish(np.float64(theta[0]))
            joint_2.publish(np.float64(theta[1]))
            joint_3.publish(np.float64(theta[2]))
            joint_4.publish(np.float64(theta[3]))
            joint_5.publish(np.float64(theta[4]))
            joint_6.publish(np.float64(theta[5]))

    print("Go stand off pos")

    return theta, gripper_state

def main():
    """
    dt : delta time
    Kp : P-gain
    Ki : I-gain
    x_ref, y_ref, z_ref, rx_ref, ry_ref, rz_ref : End-effector's reference pos & orientaion
    x_ref, y_ref, z_ref, rx_ref, ry_ref, rz_ref : End-effector's simulation pos & orientaion
    trajectory = Trajectory of maniuplator
    """
    # global robot_state
    twist = Twist()
    DEG2RAD = lambda deg: deg*np.pi/180

    dt = 0.01
    Kp = 18
    Ki = 3.5
    
    Tsc_i = np.array([[1,0,0,3],[0,1,0,0],[0,0,1,0.06],[0,0,0,1]])
    Tsc_f = np.array([[0,-1,0,3],[1,0,0,1.5],[0,0,1,0.06],[0,0,0,1]])
    Tce_g = get_X_d(0.0, 0.0, 0.06, 0.0, np.pi/2, np.pi/2)
    Tce_s = get_X_d(0.0, 0.0, 0.2, 0.0, np.pi/2,  np.pi/2)

    traj = [[np.nan]*2]
    traj[0][0], traj[0][1] = TrajectoryGenerator(Tse_i, Tsc_i, Tsc_f, Tce_g, Tce_s)
    robot_state = robot_state_init
    i = 0
    try :
        for i in range(2):

            if i == 0:
                trajectory = traj[0][0]
            elif i ==1:
                trajectory = traj[0][1]

            for j in range(len(trajectory)-1):

                X, J_e = get_Jacobian(robot_state)
                X_d = trajectory[j]
                X_d_next = trajectory[j+1]

                u_th_d, X_err = PI_control(X_d, X_d_next, X, Kp, Ki, dt, J_e)
                robot_state, vel = Next_step(u_th_d, robot_state, dt)

                twist.linear.x = vel[1]
                twist.linear.y = vel[2]
                twist.angular.z = vel[0]
                pub.publish(twist)

                joint_1.publish(np.float64(robot_state[7]))
                joint_2.publish(np.float64(robot_state[8]))
                joint_3.publish(np.float64(robot_state[9]))
                joint_4.publish(np.float64(robot_state[10]))
                joint_5.publish(np.float64(robot_state[11]))
                joint_6.publish(np.float64(robot_state[12]))

                rate.sleep()

                gripper = Gripper_close(gripper)
                time.sleep(3)

                last_chassis_pos = robot_state[:3]
            
                if i == 0 :
                    twist.linear.x = 0.
                    twist.linear.y = 0.
                    twist.angular.z = 0.
                    pub.publish(twist)
                    thetalist_current, gripper = Put_in_the_basket(thetalist_current, gripper, last_chassis_pos, 0)
                elif i == 1 :
                    twist.linear.x = 0.
                    twist.linear.y = 0.
                    twist.angular.z = 0.
                    pub.publish(twist)
                    thetalist_current, gripper = Put_in_the_basket(thetalist_current, gripper, last_chassis_pos, 1)
                else :
                    continue
            
    except KeyboardInterrupt:

        print("End mobile manipulation")
    
   
if __name__ == '__main__':

    main()
