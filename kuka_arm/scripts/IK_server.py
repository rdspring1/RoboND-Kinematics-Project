#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import math
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

# Define DH param symbols
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alphaG = symbols('alpha0:7')
a0, a1, a2, a3, a4, a5, aG = symbols('a0:7')
d1, d2, d3, d4, d5, d6, dG = symbols('d1:8')
q1, q2, q3, q4, q5, q6, qG = symbols('q1:8')

# Joint angle symbols
theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta0:6')
      
# Modified DH params
a12 = 0.35
a23 = 1.25
a34 = -0.054
d01 = 0.75
d34 = 1.50
d5G = 0.303

s = {alpha0:     0,  a0:   0, d1: d01,
     alpha1: -pi/2,  a1: a12, d2:   0, q2: q2-pi/2,
     alpha2:     0,  a2: a23, d3:   0,
     alpha3: -pi/2,  a3: a34, d4: d34,
     alpha4:  pi/2,  a4:   0, d5:   0,
     alpha5: -pi/2,  a5:   0, d6:   0,
     alphaG:     0,  aG:   0, dG: d5G, qG: 0}

# Define Modified DH Transformation matrix
# Create individual transformation matrices
T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
               [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
               [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
               [                   0,                   0,            0,               1]])

T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
               [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
               [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
               [                   0,                   0,            0,               1]])

T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
               [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
               [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
               [                   0,                   0,            0,               1]])

T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
               [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
               [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
               [                   0,                   0,            0,               1]])

T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
               [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
               [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
               [                   0,                   0,            0,               1]])

T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
               [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
               [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
               [                   0,                   0,            0,               1]])

T6_G = Matrix([[             cos(qG),            -sin(qG),            0,              aG],
               [ sin(qG)*cos(alphaG), cos(qG)*cos(alphaG), -sin(alphaG), -sin(alphaG)*dG],
               [ sin(qG)*sin(alphaG), cos(qG)*sin(alphaG),  cos(alphaG),  cos(alphaG)*dG],
               [                   0,                   0,            0,               1]])

# Substitute DH parameters into Transformation Matrices
T0_1 = T0_1.subs(s)
T1_2 = T1_2.subs(s)
T2_3 = T2_3.subs(s)
T3_4 = T3_4.subs(s)
T4_5 = T4_5.subs(s)
T5_6 = T5_6.subs(s)
T6_G = T6_G.subs(s)

# Composition of Homogenous Transforms
T0_2 = simplify(T0_1 * T1_2)
T0_3 = simplify(T0_2 * T2_3)
T0_4 = simplify(T0_3 * T3_4)
T0_5 = simplify(T0_4 * T4_5)
T0_6 = simplify(T0_5 * T5_6)
T0_G = simplify(T0_6 * T6_G)

# Numerically evaluate transforms (compare against output from tf_echo)
#print("T0_1 = ", T0_1.evalf(subs={q1: 0, q2: 0, q3:0, q4:0, q5:0, q6:0}))
#print("T0_2 = ", T0_2.evalf(subs={q1: 0, q2: 0, q3:0, q4:0, q5:0, q6:0}))
#print("T0_3 = ", T0_3.evalf(subs={q1: 0, q2: 0, q3:0, q4:0, q5:0, q6:0}))
#print("T0_4 = ", T0_4.evalf(subs={q1: 0, q2: 0, q3:0, q4:0, q5:0, q6:0}))
#print("T0_5 = ", T0_5.evalf(subs={q1: 0, q2: 0, q3:0, q4:0, q5:0, q6:0}))
#print("T0_6 = ", T0_6.evalf(subs={q1: 0, q2: 0, q3:0, q4:0, q5:0, q6:0}))
#print("T0_G = ", T0_G.evalf(subs={q1: 0, q2: 0, q3:0, q4:0, q5:0, q6:0}))

# Gripper_Link - URDF to DH Convention - Correction
R_z = Matrix([[     cos(pi),     -sin(pi),              0,     0],
              [     sin(pi),      cos(pi),              0,     0],
              [           0,            0,              1,     0],
              [           0,            0,              0,     1]])

R_y = Matrix([[     cos(-pi/2),         0,     sin(-pi/2),     0],
              [              0,         1,              0,     0],
              [    -sin(-pi/2),         0,     cos(-pi/2),     0],
              [              0,         0,              0,     1]])
R_correction = simplify(R_z * R_y)

# Total Homogenous Transformation between Base_Link and Gripper_Link with Orientation Correction
T_total = simplify(T0_G * R_correction)

'''
# Calculate R_RPY using roll, pitch, yaw
sym_alpha, sym_beta, sym_gamma = symbols('sym0:3')

R_x = Matrix([[ 1,                     0,               0],
              [ 0,        cos(sym_alpha), -sin(sym_alpha)],
              [ 0,        sin(sym_alpha),  cos(sym_alpha)]])

R_y = Matrix([[ cos(sym_beta),         0,  sin(sym_beta)],
              [              0,        1,               0],
              [-sin(sym_beta),         0,  cos(sym_beta)]])

R_z = Matrix([[ cos(sym_gamma), -sin(sym_gamma),        0],
              [ sin(sym_gamma),  cos(sym_gamma),        0],
              [             0,                0,        1]])

R_RPY = simplify(R_z * R_y * R_x)
empty = Matrix([[0.], [0.], [0.]])
base = Matrix([[ 0., 0., 0., 1.]])
T_RPY = R_RPY.row_join(empty).col_join(base)
T_RPY_sub = T_RPY.evalf(subs={sym_alpha : roll, sym_beta : pitch, sym_gamma : yaw}) * R_correction
'''

def distance(x, y):
    return math.sqrt( sum( [(left - right) ** 2.0 for left, right in zip(x,y)] ) )

def diff(sym, prev_theta):
    return sum([abs(left.evalf() - right) for left, right in zip(sym, prev_theta)])

def equal(value, other):
    diff = abs(value - other)
    if diff < 1e-1:
        return True
    else:
        return False

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))

    # Track previous angles for joint 4, 5, 6 - Non-Singular
    prev_theta = (0.0, 0.0, 0.0)

    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            # Calculate joint angles using Geometric IK method
            R_RPY = Matrix(tf.transformations.euler_matrix(roll,pitch,yaw, axes='sxyz'))
            R_RPY[0,3] = px
            R_RPY[1,3] = py
            R_RPY[2,3] = pz
            T_RPY_sub = R_RPY * R_correction

            # Calculate Wrist Center
            wx = px - d5G * T_RPY_sub[0,2]
            wy = py - d5G * T_RPY_sub[1,2]
            wz = pz - d5G * T_RPY_sub[2,2]

            # Calculate Position, q1, q2, q3
            # ELBOW - DOWN
            height = wz - d01
            width = sqrt(math.pow(wx, 2.0) + math.pow(wy, 2.0)) - a12
            l = sqrt(math.pow(height, 2.0) + math.pow(width, 2.0))

            alpha = atan2(height, width)
            beta = (math.pow(a23, 2.0) + math.pow(d34, 2.0) - math.pow(l, 2.0)) / (2.0 * a23 * d34)
            gamma = (math.pow(l, 2.0) + math.pow(a23, 2.0) - math.pow(d34, 2.0)) / (2.0 * a23 * l)

            theta3_correction = 0.03459
            sym_theta1 = atan2(wy, wx)
            sym_theta2 = pi/2 - (alpha + atan2(sqrt(1 - math.pow(gamma, 2.0)), gamma))
            sym_theta3 = pi/2 - theta3_correction - atan2(sqrt(1 - math.pow(beta, 2.0)), beta)

            theta1 = sym_theta1.evalf()
            theta2 = sym_theta2.evalf()
            theta3 = sym_theta3.evalf()

            # Calculate T3_6
            T0_3_sub = T0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            T3_6 = Transpose(T0_3_sub) * R_RPY

            r10 = T3_6[1,0]
            r00 = T3_6[0,0]
            r11 = T3_6[1,1]
            r12 = T3_6[1,2]
            r20 = T3_6[2,0]

            # Using q1, q2, q3, calculate Orientation, q4, q5, q6
            if equal(r10,1) and equal(r00,0) and equal(r11,0) and equal(r12,0) and equal(r20,0):
                #print("singular")

                # Use previous angles for joints 4+6 and angle estimate (theta4 + theta6)
                q46 = atan2(T3_6[0,1], T3_6[2,1])
                theta5 = 0
                theta4 = prev_theta[0]
                theta6 = q46 - theta4
                #current_theta = (theta4, theta5, theta6.evalf())
                #current_diff = sum([abs(left - right) for left, right in zip(current_theta, prev_theta)])
            else:
                #print("non-singular")

                cos_q5 = r10
                sin_q5 = sqrt(1 - math.pow(cos_q5, 2.0))

                pos_sym = (atan2(r20, -r00), atan2(sin_q5, cos_q5), atan2(r11, r12))
                pos_diff = diff(pos_sym, prev_theta)

                neg_sym = (atan2(-r20, r00), atan2(-sin_q5, cos_q5), atan2(-r11, -r12))
                neg_diff = diff(neg_sym, prev_theta)

                threshold = 1.5
                current_diff = min(pos_diff, neg_diff)
                #print(pos_diff, neg_diff)
                if pos_diff < neg_diff:
                    theta4, theta5, theta6 = [sym.evalf() for sym in pos_sym]
                else:
                    theta4, theta5, theta6 = [sym.evalf() for sym in neg_sym]

            # Store angles for joints 4, 5, 6
            prev_theta = (theta4, theta5, theta6)

            # Calculate distance between desired and output end-effector poses
            new_pos = T_total.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4:theta4, q5:theta5, q6:theta6})
            npx, npy, npz = [new_pos[0,3], new_pos[1,3], new_pos[2,3]]
            print(distance((px, py, pz), (npx, npy, npz)))

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
