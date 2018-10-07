from sympy import *
from time import time
from mpmath import radians, pi
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}

def cosRuleA(a, b, c):
    return acos((b*b+c*c-a*a)/2/b/c)

def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()

    ########################################################################################

    # Create symbols
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    z0, z1, z2, z3, z4, z5, z6 = symbols('z0:7')

    # Create Modified DH parameters
    s = {
	z0:     0, a0:      0, d1:   0.75, q1:      q1,
	z1:-pi/2., a1:   0.35, d2:      0, q2:q2-pi/2.,
	z2:     0, a2:   1.25, d3:      0, q3:      q3,
	z3:-pi/2., a3: -0.054, d4:    1.5, q4:      q4,
	z4: pi/2., a4:      0, d5:      0, q5:      q5,
	z5:-pi/2., a5:      0, d6:      0, q6:      q6,
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
    r, p, y = symbols('r p y')
    R_z = Matrix([
            [cos(y), -sin(y), 0],
            [sin(y),  cos(y), 0],
            [     0,       0, 1]
            ]) # pi
    R_y = Matrix([
            [ cos(p), 0, sin(p)],
            [      0, 1,      0],
            [-sin(p), 0, cos(p)]
            ]) # -pi/2
    R_x = Matrix([
            [1,      0,       0],
            [0, cos(r), -sin(r)],
            [0, sin(r),  cos(r)]
            ]) # 0
    Rot = R_z * R_y * R_x
    R_cor = R_z.subs(y, pi) * R_y.subs(p, -pi/2.)
    Rot = simplify(Rot)
    print(Rot)
    Rot = Rot * R_cor
    #T_tot = simplify(T0_G * R_cor)
    #T_tot = T0_G * R_cor


    # ee pos
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x, req.poses[x].orientation.y,
        req.poses[x].orientation.z, req.poses[x].orientation.w])

    ee = Matrix([[px],
                 [py],
                 [pz]])
    wo = Rot.subs({'y':yaw, 'p':pitch, 'r': roll})
    wc = ee - s[d7] * wo[:,2]
    sa = s[d4]
    sb = sqrt(pow(sqrt(wc[0]*wc[0]+wc[1]*wc[1]) - s[a1], 2) + pow(wc[2]-s[d1], 2))
    sc = s[a2]
    aa = cosRuleA(sa, sb, sc)
    ab = cosRuleA(sb, sa, sc)
    ac = cosRuleA(sc, sa, sb)

    theta1 = atan2(wc[1], wc[0])
    theta2 = pi/2 - aa - atan2(wc[2] - s[d1], sqrt(wc[0]*wc[0]+wc[1]*wc[1]) - s[a1])
    theta3 = pi/2 - (ab+0.036)

    R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
    R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
    R3_6 = R0_3.inv("LU") * wo

    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2]+R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
    theta6 = atan2(-R3_6[1,1], R3_6[1,0])

    ##
    ########################################################################################

    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!
    fk = T0_G.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6:theta6})

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [wc[0],wc[1],wc[2]] # <--- Load your calculated WC values in this array
    your_ee = [fk[0,3],fk[1,3],fk[2,3]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 2

    test_code(test_cases[test_case_number])
