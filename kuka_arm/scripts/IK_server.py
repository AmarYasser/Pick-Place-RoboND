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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
	#
	#
	# Create Modified DH parameters
	#
	#
	# Define Modified DH Transformation matrix
	#
	#
	# Create individual transformation matrices
	#
	#
	# Extract rotation matrices from the transformation matrices
	#
	#
        ###
	#Forward Kinematics for Kuka210 RoboND

	#import required liberaries


	# Create symbols for joint variables
	q1, q2, q3, q4 ,q5,q6 ,q7 = symbols('q1:8')
	d1, d2, d3, d4 ,d5, d6, d7 = symbols('d1:8')	
	a0, a1, a2, a3,a4, a5, a6 = symbols('a0:7')
	alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

		# DH Parameters
	DH = {alpha0: 0   ,  a0:        0, d1:  0.75, 
	     	      alpha1: -pi/2,  a1:     0.35, d2:     0,q2: q2-pi/2,  
	     	      alpha2: 0    ,  a2:     1.25, d3:     0,
	     	      alpha3: -pi/2,  a3:   -0.054, d4:   1.5,
	     	      alpha4:  pi/2,  a4:        0, d5:     0,
	     	      alpha5: -pi/2,  a5:        0, d6:     0,
	       	      alpha6: 0    ,  a6:        0, d7: 0.303,q7: 0}

			#### Homogeneous Transforms
	T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
			       [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
			       [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
			       [                   0,                   0,            0,               1]])
	T0_1 = T0_1.subs(DH)

	T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
			       [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
			       [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
			       [                   0,                   0,            0,               1]])
	T1_2 = T1_2.subs(DH)

	T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
			       [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
			       [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
			       [                   0,                   0,            0,               1]])
	T2_3 = T2_3.subs(DH)

	T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
			       [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
			       [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
			       [                   0,                   0,            0,               1]])
	T3_4 = T3_4.subs(DH)

	T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
			       [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
			       [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
			       [                   0,                   0,            0,               1]])
	T4_5 = T4_5.subs(DH)

	T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
			       [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
			       [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
			       [                   0,                   0,            0,               1]])
	T5_6 = T5_6.subs(DH)

	T6_g = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
			       [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
			       [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
			       [                   0,                   0,            0,               1]])
	T6_g = T6_g.subs(DH)

		#HT for each frame relative to the base
	T0_2= simplify (T0_1 *T1_2)
	T0_3= simplify (T0_2 *T2_3)
	T0_4= simplify (T0_3 *T3_4)
	T0_5= simplify (T0_4 *T4_5)
	T0_6= simplify (T0_5 *T5_6)
	T0_g= simplify (T0_6 *T6_g)

		#Correlation for the girpper rotation 
		#we dirst rotate about z-axis by 180 deg and about y-axis by -90 deg "interisicly"
		# define the HTs to perform the required rotaions considering the transilation = [0 0 0 1]
	Rot_y = Matrix([[ cos(-pi/2),        0,  sin(-pi/2),  0],
			      [          0,        1,           0,  0],
			      [-sin(-pi/2),        0,  cos(-pi/2),  0],
			      [          0,        0,        0,    1]])

	Rot_z = Matrix([[ cos(pi), -sin(pi),        0, 0],
			      [ sin(pi),  cos(pi),        0, 0],
			      [ 0,              0,        1, 0],
			      [ 0,              0,        0,  1]])

	Rcor= simplify(Rot_z*Rot_y)
		#the main HT after correction
	T0_G=simplify(T0_g*Rcor)

		#TEST_FK
		#Prit the output from the HT when all the variables are 0 

		#print('T0_1= ', T0_1.evalf(subs={q1:0,q2:0,q3:0,q4:0,q5:0,q6:0}))
		#print('T0_2= ', T0_2.evalf(subs={q1:0,q2:0,q3:0,q4:0,q5:0,q6:0}))
		#print('T0_3= ', T0_3.evalf(subs={q1:0,q2:0,q3:0,q4:0,q5:0,q6:0}))
		#print('T0_4= ', T0_4.evalf(subs={q1:0,q2:0,q3:0,q4:0,q5:0,q6:0}))
		#print('T0_5= ', T0_5.evalf(subs={q1:0,q2:0,q3:0,q4:0,q5:0,q6:0}))
		#print('T0_6= ', T0_6.evalf(subs={q1:0,q2:0,q3:0,q4:0,q5:0,q6:0}))
		#print('T0_G= ', T0_G.evalf(subs={q1:0,q2:0,q3:0,q4:0,q5:0,q6:0}))

		#print('T0_1= ', T0_1.evalf(subs={q1:1.05,q2:1.45,q3:-3.21,q4:-1.5,q5:-0.83,q6:-4.96}))
		#print('T0_2= ', T0_2.evalf(subs={q1:1.05,q2:1.45,q3:-3.21,q4:-1.5,q5:-0.83,q6:-4.96}))
		#print('T0_3= ', T0_3.evalf(subs={q1:1.05,q2:1.45,q3:-3.21,q4:-1.5,q5:-0.83,q6:-4.96}))
		#print('T0_4= ', T0_4.evalf(subs={q1:1.05,q2:1.45,q3:-3.21,q4:-1.5,q5:-0.83,q6:-4.96}))
		#print('T0_5= ', T0_5.evalf(subs={q1:1.05,q2:1.45,q3:-3.21,q4:-1.5,q5:-0.83,q6:-4.96}))
		#print('T0_6= ', T0_6.evalf(subs={q1:1.05,q2:1.45,q3:-3.21,q4:-1.5,q5:-0.83,q6:-4.96}))
		#print('T0_G= ', T0_G.evalf(subs={q1:1.05,q2:1.45,q3:-3.21,q4:-1.5,q5:-0.83,q6:-4.96}))
        

	
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

            ### Your IK code here
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #
	    #
            RotY = Matrix([[ cos(pitch),        0,  sin(pitch),  0],
			      [          0,        1,           0,  0],
			      [-sin(pitch),        0,  cos(pitch),  0],
			      [          0,        0,        0,    1]])

            RotZ = Matrix([[ cos(yaw), -sin(yaw),        0, 0],
			      [ sin(yaw),  cos(yaw),        0, 0],
			      [ 0,              0,        1, 0],
			      [ 0,              0,        0,  1]])
            RotX= Matrix([[ 1, 0,         0,          0],
			      [ 0, 1,         0,          0],
			      [ 0, 0,cos(roll), -sin(roll)],
			      [ 0, 0,sin(roll),  cos(roll)]])

            Rrpy= RotZ*RotY*RotX*Rcor
	
		#Extract rotaion for z-axis	
            nx= Rrpy[0,2]
            ny= Rrpy[1,2]
            nz= Rrpy[2,2]
	#Find the position of WC	
            l = 0.303 	# length from WC to EE			
            wx= px-(l)*nx
            wy= py-(l)*ny
            wz= pz-(l)*nz
	    # Calculate joint angles using Geometric IK method
	    #
	    #
            ###
		#define some varibles from trigonometry
            A=1.50097 # calculated petha. theory with sides (1.5,0.054)
            C=1.25
            B=sqrt( (wz-0.75)**2 + (sqrt(wx**2+wy**2)-0.35)**2)
            A_angle=acos((B**2+C**2-A**2)/(2*B*C))
            x_angle=atan2((wz-0.75) , (sqrt(wx**2+wy**2)-0.35) ) # angle between B and x-axis
				

            theta1= atan2(wy,wx)
            theta2= pi/2-x_angle-A_angle
			
            t_angle= atan2(0.054,1.5)
            B_angle=acos((A**2+C**2-B**2)/(2*A*C))
            theta3= pi/2-B_angle-t_angle
		

		#Extract rotations from HT 
            R0_1=T0_1[0:3,0:3]
            R1_2=T1_2[0:3,0:3]
            R2_3=T2_3[0:3,0:3]

		#Perfom the rotation from base to 3
            R0_3=R0_1*R1_2*R2_3
            R0_3= R0_3.evalf(subs={q1:theta1 , q2:theta2 , q3:theta3}) 
		
		#find the rotation from 3 to 6
            R3_6= R0_3.inv("LU") * Rrpy[0:3,0:3]
		
		#calculate thetas 4,5,6 by using euler

            theta4=atan2( R3_6[2,2], -R3_6[0,2])
            theta5=atan2( sqrt(R3_6[0,2]**2 +R3_6[2,2]**2) , R3_6[1,2] )
            theta6=atan2( R3_6[1,1] , R3_6[1,0] )

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
