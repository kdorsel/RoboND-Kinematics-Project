## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code.


[//]: # (Image References)

[image1]: ./misc_images/dh.jpg
[image2]: ./misc_images/screen1.png
[image3]: ./misc_images/screen2.png
[image4]: ./misc_images/screen3.png
[image5]: ./misc_images/theta.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

DH table create based on the drawing below. The axis orientations for X and Z were taken from the course to match up. Distance d and a were taken from the Kuka kr210 drawing available online.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - 90 | 0.35 | 0 | q2 - pi/2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  - 90 | -0.055 | 1.5 | q4
4->5 | 90 | 0 | 0 | q5
5->6 | - 90 | 0 | 0 | q6
6->G | 0 | 0 | 0.303 | 0

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The following matrices were created based for each of the lines in the DH table. The matrix itself is the homogeneous transform matrix as create in class.


```
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
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Theta1-3:
![alt text][image5]

Theta4-6:
These were taken from the project guide video directly.



### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

A lot of the code is based on the class and the implementation is a near copy. A cosine law function was added to ease the calculation of theta1-3. Also values were taken directly from the DH table (a and d) instead of hard coding them. This makes it easier to change the robot, by only having to change the DH table.

Obviously more work can be done with smoothing out the motion and making the motion more intelligent by looking at previous and future joint positions.

Currently singularities are not supported and that could be something to work on in the future. I know Kuka already has this built into their software and use S and T variables (Status and Turn) to fully define a position.

Screenshots below.

![alt text][image2]
![alt text][image3]
![alt text][image4]
