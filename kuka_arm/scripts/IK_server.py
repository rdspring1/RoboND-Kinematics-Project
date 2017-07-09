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
T6_G = T5_6.subs(s)

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

# Calculate R_RPY using roll, pitch, yaw
sym_alpha, sym_beta, sym_gamma = symbols('sym0:3')
R_x = Matrix([[ cos(sym_alpha), -sin(sym_alpha),        0],
  [ sin(sym_alpha),  cos(sym_alpha),        0],
  [             0,                0,        1]])

R_y = Matrix([[ cos(sym_beta),         0,  sin(sym_beta)],
  [              0,        1,               0],
  [-sin(sym_beta),         0,  cos(sym_beta)]])

R_z = Matrix([[ 1,                     0,               0],
  [ 0,        cos(sym_gamma), -sin(sym_gamma)],
  [ 0,        sin(sym_gamma),  cos(sym_gamma)]])

R_RPY = simplify(R_z * R_y * R_x)
empty = Matrix([[0.], [0.], [0.]])
base = Matrix([[ 0., 0., 0., 1.]])
T_RPY = R_RPY.row_join(empty).col_join(base)

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
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
     
            # TODO Calculate joint angles using Geometric IK method
            T_RPY_sub = T_RPY.evalf(subs={sym_alpha : roll, sym_beta : pitch, sym_gamma : yaw})

            # Calculate Wrist Center
            wx = px - d5G * T_RPY_sub[2,0]
            wy = py - d5G * T_RPY_sub[2,1] 
            wz = pz - d5G * T_RPY_sub[2,2]

            # Calculate Position, q1, q2, q3
            # ELBOW - DOWN
            height = wz - d01
            width = sqrt(math.pow(wx, 2.0) + math.pow(wy, 2.0)) - a12
            l = sqrt(math.pow(height, 2.0) + math.pow(width, 2.0))

            alpha = atan2(height, width)
            beta = (math.pow(a23, 2.0) + math.pow(d34, 2.0) - math.pow(l, 2.0)) / (2 * a23 * d34)
            gamma = (math.pow(l, 2.0) + math.pow(a23, 2.0) + math.pow(d34, 2.0)) / (2 * a23 * l)

            theta1 = atan2(wy, wx)
            theta2 = alpha - gamma
            theta3 = pi - beta
            
            # Calculate T3_6
            T0_3_sub = T0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            T3_6 = Transpose(T0_3_sub) * T_RPY_sub

            # Using q1, q2, q3, calculate Orientation, q4, q5, q6
            r13 = T3_6[0,2]
            r23 = T3_6[1,2]
            r33 = T3_6[2,2]

            if r13 == 0.0 and r23 == 0.0:
                # Singular - Arbitrarily set theta 4 to pi/2
                print("singular")
                theta5 = 0

                T3_6 = T3_6.evalf(subs={q5: theta5})
                r21 = T3_6[1,0]
                r11 = T3_6[0,0]
                theta46 = atan2(r21, r11)
                theta4 = pi / 2
                theta6 = theta46 - theta4
            else:
                # Non-Singular
                print("non-singular")
                theta5 = atan2(sqrt(1-math.pow(r33, 2.0)), r33) 

                T3_6 = T3_6.evalf(subs={q5: theta5})
                r13 = T3_6[0,2]
                r23 = T3_6[1,2]
                r32 = T3_6[2,1]
                r31 = T3_6[2,0]

                if theta5 > 0.0:
                    theta4 = atan2( r23,  r13)
                    theta6 = atan2( r32, -r31)
                else:
                    theta4 = atan2(-r23, -r13)
                    theta6 = atan2(-r32,  r31)
            
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
