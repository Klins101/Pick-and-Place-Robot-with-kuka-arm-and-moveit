"""
ROS node for Inverse Kinematic analyis of the KUKA KR210 robot arm.
"""
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
import numpy as np 
from numpy.linalg import inv
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt


def get_DH_Table():
    theta1, theta2, theta3, theta4, theta5, theta6 = 0., 0., 0., 0., 0., 0.
    dh = {'alpha0':     0,  'a0':      0,  'd1':  0.75,  'theta1':  theta1,
          'alpha1': -np.pi/2,  'a1':   0.35,  'd2':     0,  'theta2':  theta2,
          'alpha2':     0,  'a2':   1.25,  'd3':     0,  'theta3':  theta3,
          'alpha3': -np.pi/2,  'a3': -0.054,  'd4':  1.50,  'theta4':  theta4,
          'alpha4':  np.pi/2,  'a4':      0,  'd5':     0,  'theta5':  theta5,
          'alpha5': -np.pi/2,  'a5':      0,  'd6':     0,  'theta6':  theta6,
          'alpha6':     0,  'a6':      0,  'dG': 0.303,  'thetaG':       0}
    return dh


def get_Rx(theta):
    Rx = np.matrix([[1,          0,           0],
                 [0, np.cos(theta), -np.sin(theta)],
                 [0, np.sin(theta),  np.cos(theta)]])
    return Rx


def get_Ry(theta):
    Ry = np.matrix([[np.cos(theta),  0, np.sin(theta)],
                 [         0,  1,          0],
                 [-np.sin(theta), 0, np.cos(theta)]])
    return Ry


def get_Rz(theta):
    Rz = np.matrix([[np.cos(theta), -np.sin(theta), 0],
                 [np.sin(theta),  np.cos(theta), 0],
                 [         0,           0, 1]])
    return Rz


def get_TF(alpha, a, d, theta):
    Tf = np.matrix([
        [           np.cos(theta),            -np.sin(theta),            0,              a],
        [np.sin(theta)*np.cos(alpha),  np.cos(theta)*np.cos(alpha),  -np.sin(alpha),  -np.sin(alpha)*d],
        [np.sin(theta)*np.sin(alpha),  np.cos(theta)*np.sin(alpha),   np.cos(alpha),   np.cos(alpha)*d],
        [                    0,                      0,            0,              1]
     ])
    return Tf


def get_ee_pose(pose_msg):
    ee_x = pose_msg.position.x
    ee_y = pose_msg.position.y
    ee_z = pose_msg.position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [pose_msg.orientation.x, pose_msg.orientation.y,
         pose_msg.orientation.z, pose_msg.orientation.w]
        )
    position = (ee_x, ee_y, ee_z)
    orientation = (roll, pitch, yaw)
    return position, orientation


def get_R_EE(ee_pose):
    roll, pitch, yaw = ee_pose[1]
    R_ee = get_Rz(yaw) * get_Ry(pitch) * get_Rx(roll)
    Rerror = get_Rz(pi) * get_Ry(-pi/2)
    R_ee = R_ee * Rerror
    return R_ee


def get_WC(dh, R_ee, ee_pose):
    ee_x, ee_y, ee_z = ee_pose[0]
    EE_P = np.matrix([[ee_x],
                   [ee_y],
                   [ee_z]])
    Z_ee = R_ee[:, 2]
    Wc = EE_P - dh['dG']*Z_ee

    return Wc


def get_joints1_2_3(dh, Wc):
    wcx, wcy, wcz = Wc[0], Wc[1], Wc[2]

    theta1 = np.arctan2(wcy, wcx)
    wcz_j2 = wcz - dh['d1']                            
    wcx_j2 = np.sqrt(wcx**2 + wcy**2) - dh['a1']         
    side_a = round(np.sqrt((dh['d4'])**2 + (dh['a3'])**2), 7)  
    side_b = np.sqrt(wcx_j2**2 + wcz_j2**2)                    
    side_c = dh['a2']                                       
    angleA = np.arccos((side_b**2 + side_c**2 - side_a**2) / (2*side_b*side_c))
    angleB = np.arccos((side_a**2 + side_c**2 - side_b**2) / (2*side_a*side_c))
    angleC = np.arccos((side_a**2 + side_b**2 - side_c**2) / (2*side_a*side_b))
    angle_sag = round(np.arctan2(abs(dh['a3']), dh['d4']), 7)
    theta2 = np.pi/2 - angleA - np.arctan2(wcz_j2, wcx_j2)
    theta3 = np.pi/2 - (angleB + angle_sag)
    return theta1, theta2, theta3


def get_joints4_5_6(dh, R_ee, theta1, theta2, theta3):
    T0_1 = get_TF(dh['alpha0'], dh['a0'], dh['d1'], dh['theta1'])
    T1_2 = get_TF(dh['alpha1'], dh['a1'], dh['d2'], dh['theta2'])
    T2_3 = get_TF(dh['alpha2'], dh['a2'], dh['d3'], dh['theta3'])
    R0_1 = T0_1[0:3, 0:3]
    R1_2 = T1_2[0:3, 0:3]
    R2_3 = T2_3[0:3, 0:3]
    R0_3 = R0_1 * R1_2 * R2_3
    R3_6 = inv(np.array(R0_3, dtype='float')) * R_ee  # b/c R0_6 == R_ee = R0_3*R3_6
    r21 = R3_6[1, 0]  
    r22 = R3_6[1, 1]  
    r13 = R3_6[0, 2] 
    r23 = R3_6[1, 2]  
    r33 = R3_6[2, 2]  
    theta4 = np.arctan2(r33, -r13)
    theta5 = np.arctan2(np.sqrt(r13**2 + r33**2), r23)
    theta6 = np.arctan2(-r22, r21)
    return theta4, theta5, theta6


def plot_EE(received_ee_points, fk_ee_points, ee_errors):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    data1 = np.array(received_ee_points)
    x1, y1, z1 = data1.T
    ax.scatter(x1, y1, z1, c='blue', s=50, marker='o')
    ax.plot(x1, y1, z1, c='blue', label='rec_ee')
    data2 = np.array(fk_ee_points)
    x2, y2, z2 = data2.T
    ax.scatter(x2, y2, z2, c='orange', s=50, marker='s')
    ax.plot(x2, y2, z2, c='orange', label='fk_ee')
    data3 = np.array(ee_errors)
    x3, y3, z3 = data3.T
    ax.scatter(x3, y3, z3, c='magenta', s=50, marker='^')
    ax.plot(x3, y3, z3, c='magenta', label='ee_error')
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.xaxis.label.set_color('red')
    ax.yaxis.label.set_color('green')
    ax.zaxis.label.set_color('blue')
    plt.legend()
    plt.show()


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses" % len(req.poses))
    if len(req.poses) < 1:
        print("No pose received")
        return -1
    else:
        dh = get_DH_Table()
        joint_trajectory_list = []
        received_ee_points = []
        fk_ee_points = []
        ee_errors = []

        # Calculating pose for each joint
        len_poses = len(req.poses)
        for x in xrange(0, len_poses):
            joint_trajectory_point = JointTrajectoryPoint()
            # inverse kinematics
            ee_pose = get_ee_pose(req.poses[x])
            received_ee_points.append(ee_pose[0])
            R_ee = get_R_EE(ee_pose)
            Wc = get_WC(dh, R_ee, ee_pose)

            theta1, theta2, theta3 = get_joints1_2_3(dh, Wc)
            dh['theta1'] = theta1
            dh['theta2'] = theta2-np.pi/2  
            dh['theta3'] = theta3

            theta4, theta5, theta6 = get_joints4_5_6(dh, R_ee, theta1, theta2, theta3)
            dh['theta4'] = theta4
            dh['theta5'] = theta5
            dh['theta6'] = theta6

            joint_trajectory_point.positions = [theta1, theta2, theta3,
                                                theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

            def calculate_FK():
                """Calculate Forward Kinematics for verifying joint angles."""
                # Compute individual transforms between adjacent links
                # T(i-1)_i = Rx(alpha(i-1)) * Dx(alpha(i-1)) * Rz(theta(i)) * Dz(d(i))
                T0_1 = get_TF(dh['alpha0'], dh['a0'], dh['d1'], dh['theta1'])
                T1_2 = get_TF(dh['alpha1'], dh['a1'], dh['d2'], dh['theta2'])
                T2_3 = get_TF(dh['alpha2'], dh['a2'], dh['d3'], dh['theta3'])
                T3_4 = get_TF(dh['alpha3'], dh['a3'], dh['d4'], dh['theta4'])
                T4_5 = get_TF(dh['alpha4'], dh['a4'], dh['d5'], dh['theta5'])
                T5_6 = get_TF(dh['alpha5'], dh['a5'], dh['d6'], dh['theta6'])
                T6_ee = get_TF(dh['alpha6'], dh['a6'], dh['dG'], dh['thetaG'])
                # Create overall transform between base frame and EE by
                # composing the individual link transforms
                T0_ee = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_ee
                fk_ee = [T0_ee[0, 3], T0_ee[1, 3], T0_ee[2, 3]]
                fk_ee_points.append([round(fk_ee[0].item(0), 8),
                                     round(fk_ee[1].item(0), 8),
                                     round(fk_ee[2].item(0), 8)])
                ee_x_e = abs(fk_ee[0] - ee_pose[0][0])
                ee_y_e = abs(fk_ee[1] - ee_pose[0][1])
                ee_z_e = abs(fk_ee[2] - ee_pose[0][2])
                ee_errors.append([round(ee_x_e.item(0), 8),
                                  round(ee_y_e.item(0), 8),
                                  round(ee_z_e.item(0), 8)])
           
        rospy.loginfo("Number of joint trajectory points:" +
                      " %s" % len(joint_trajectory_list))
        plot_EE(received_ee_points, fk_ee_points, ee_errors)

        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print("Ready to receive an IK request")
    rospy.spin()


if __name__ == "__main__":
    IK_server()
