#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

def cosRuleA(a, b, c):
    return acos((b*b+c*c-a*a)/2./b/c)

def t(q, d, a, z):
    return Matrix([
            [       cos(q),       -sin(q),       0,         a],
            [sin(q)*cos(z), cos(q)*cos(z), -sin(z), -sin(z)*d],
            [sin(q)*sin(z), cos(q)*sin(z),  cos(z),  cos(z)*d],
            [            0,             0,       0,         1]
            ])

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        print "Valid poses received"
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        z0, z1, z2, z3, z4, z5, z6 = symbols('z0:7')

        # Create Modified DH parameters
        s = {
	    z0:     0, a0:      0, d1:   0.75, q1:      q1,
	    z1: -pi/2, a1:   0.35, d2:      0, q2: q2-pi/2,
	    z2:     0, a2:   1.25, d3:      0, q3:      q3,
	    z3: -pi/2, a3: -0.054, d4:    1.5, q4:      q4,
	    z4:  pi/2, a4:      0, d5:      0, q5:      q5,
	    z5: -pi/2, a5:      0, d6:      0, q6:      q6,
	    z6:     0, a6:      0, d7:  0.303, q7:       0}

	    # Define Modified DH Transformation matrix
        T0_1 = Matrix([
	            [        cos(q1),        -sin(q1),        0,          a0],
	            [sin(q1)*cos(z0), cos(q1)*cos(z0), -sin(z0), -sin(z0)*d1],
	            [sin(q1)*sin(z0), cos(q1)*sin(z0),  cos(z0),  cos(z0)*d1],
	            [              0,               0,        0,           1]
	            ])
        T0_1 = T0_1.subs(s)
        T1_2 = Matrix([
	            [        cos(q2),        -sin(q2),        0,          a1],
	            [sin(q2)*cos(z1), cos(q2)*cos(z1), -sin(z1), -sin(z1)*d2],
	            [sin(q2)*sin(z1), cos(q2)*sin(z1),  cos(z1),  cos(z1)*d2],
	            [              0,               0,        0,           1]
	            ])
        T1_2 = T1_2.subs(s)
        T2_3 = Matrix([
	            [        cos(q3),        -sin(q3),        0,          a2],
	            [sin(q3)*cos(z2), cos(q3)*cos(z2), -sin(z2), -sin(z2)*d3],
	            [sin(q3)*sin(z2), cos(q3)*sin(z2),  cos(z2),  cos(z2)*d3],
	            [              0,               0,        0,           1]
	            ])
        T2_3 = T2_3.subs(s)
        T3_4 = Matrix([
	            [        cos(q4),        -sin(q4),        0,          a3],
	            [sin(q4)*cos(z3), cos(q4)*cos(z3), -sin(z3), -sin(z3)*d4],
	            [sin(q4)*sin(z3), cos(q4)*sin(z3),  cos(z3),  cos(z3)*d4],
	            [              0,               0,        0,           1]
	            ])
        T3_4 = T3_4.subs(s)
        T4_5 = Matrix([
	            [        cos(q5),        -sin(q5),        0,          a4],
	            [sin(q5)*cos(z4), cos(q5)*cos(z4), -sin(z4), -sin(z4)*d5],
	            [sin(q5)*sin(z4), cos(q5)*sin(z4),  cos(z4),  cos(z4)*d5],
	            [              0,               0,        0,           1]
	            ])
        T4_5 = T4_5.subs(s)
        T5_6 = Matrix([
	            [        cos(q6),        -sin(q6),        0,          a5],
	            [sin(q6)*cos(z5), cos(q6)*cos(z5), -sin(z5), -sin(z5)*d6],
	            [sin(q6)*sin(z5), cos(q6)*sin(z5),  cos(z5),  cos(z5)*d6],
	            [              0,               0,        0,           1]
	            ])
        T5_6 = T5_6.subs(s)
        T6_G = Matrix([
	            [        cos(q7),        -sin(q7),        0,          a6],
	            [sin(q7)*cos(z6), cos(q7)*cos(z6), -sin(z6), -sin(z6)*d7],
	            [sin(q7)*sin(z6), cos(q7)*sin(z6),  cos(z6),  cos(z6)*d7],
	            [              0,               0,        0,           1]
	            ])
        T6_G = T6_G.subs(s)

        # Create individual transformation matrices
        #T0_2 = simplify(T0_1 * T1_2)
        #T0_3 = simplify(T0_2 * T2_3)
        #T0_4 = simplify(T0_3 * T3_4)
        #T0_5 = simplify(T0_4 * T4_5)
        #T0_6 = simplify(T0_5 * T5_6)
        #T0_G = simplify(T0_6 * T6_G)
        T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

        # Extract rotation matrices from the transformation matrices
        t, u, v = symbols('t u v')
        R_z = Matrix([
                [cos(t), -sin(t), 0],
                [sin(t),  cos(t), 0],
                [     0,       0, 1]
                ]) # pi
        R_y = Matrix([
                [ cos(u), 0, sin(u)],
                [      0, 1,      0],
                [-sin(u), 0, cos(u)]
                ]) # -pi/2
        R_x = Matrix([
                [1,      0,       0],
                [0, cos(v), -sin(v)],
                [0, sin(v),  cos(v)]
                ]) # 0
        R_cor = R_z.subs(t, pi) * R_y.subs(u, -pi/2)
        Rot = R_z * R_y * R_x * R_cor

        R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]

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

            ee = Matrix([[px], [py], [pz]])
            wo = Rot.subs({'t':yaw, 'u':pitch, 'v': roll})
            wc = ee - s[d7] * wo[:,2]
            sa = s[d4]
            sb = sqrt(pow(sqrt(wc[0]*wc[0]+wc[1]*wc[1]) - s[a1], 2) + pow(wc[2]-s[d1], 2))
            sc = s[a2]
            aa = cosRuleA(sa, sb, sc)
            ab = cosRuleA(sb, sa, sc)
            ac = cosRuleA(sc, sa, sb)

            theta1 = atan2(wc[1], wc[0])
            theta2 = pi/2 - aa - atan2(wc[2] - s[d1], sqrt(wc[0]*wc[0]+wc[1]*wc[1]) - s[a1])
            theta3 = pi/2 - ab - atan2(s[a3], s[d4])

            R0_3e = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R3_6 = R0_3e.inv('LU') * wo

            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2]+R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])

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
