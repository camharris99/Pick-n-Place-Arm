"""!
Implements Forward and Inverse kinematics with DH parametrs and product of exponentials

TODO: Here is where you will write all of your kinematics functions
There are some functions to start with, you may need to implement a few more
"""

import numpy as np
# expm is a matrix exponential function
from scipy.linalg import expm
import math
import rospy


def clamp(angle):
    """!
    @brief      Clamp angles between (-pi, pi]

    @param      angle  The angle

    @return     Clamped angle
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle <= -np.pi:
        angle += 2 * np.pi
    return angle


def FK_dh(dh_params, joint_angles, link):
    """!
    @brief      Get the 4x4 transformation matrix from link to world

                TODO: implement this function

                Calculate forward kinematics for rexarm using DH convention

                return a transformation matrix representing the pose of the desired link

                note: phi is the euler angle about the y-axis in the base frame

    @param      dh_params     The dh parameters as a 2D list each row represents a link and has the format [a, alpha, d,
                              theta]
    @param      joint_angles  The joint angles of the links
    @param      link          The link to transform from

    @return     a transformation matrix representing the pose of the desired link
    """
    pass


def get_transform_from_dh(a, alpha, d, theta):
    """!
    @brief      Gets the transformation matrix T from dh parameters.

    TODO: Find the T matrix from a row of a DH table

    @param      a      a meters
    @param      alpha  alpha radians
    @param      d      d meters
    @param      theta  theta radians

    @return     The 4x4 transformation matrix.
    """
    pass


def get_euler_angles_from_T(T):
    """!
    @brief      Gets the euler angles from a transformation matrix.

                TODO: Implement this function return the 3 Euler angles from a 4x4 transformation matrix T
                If you like, add an argument to specify the Euler angles used (xyx, zyz, etc.)

    @param      T     transformation matrix

    @return     The euler angles from T.
    """

    singular_test = math.sqrt(T[0,0]*T[0,0] + T[1,0]*T[1,0])

    singular = singular_test < 1e-6

    if not singular:
        psi = math.atan2(T[2,1],T[2,2])
        theta = math.atan2(-T[2,0],singular_test)
        phi = math.atan2(T[1,0],T[0,0])
    else:
        psi = math.atan2(-T[1,2],T[1,1])
        theta = math.atan2(-T[2,0],singular_test)
        phi = 0

    return [phi,theta,psi]


def get_pose_from_T(T):
    """!
    @brief      Gets the pose from T.

                TODO: implement this function return the 6DOF pose vector from a 4x4 transformation matrix T

    @param      T     transformation matrix

    @return     The pose vector from T.
    """
    pass

def skew(x):
    skew = np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])
    return skew

def rodrigues(theta, w, v):
    w_hat = skew(w)
    S = np.hstack((w_hat,np.transpose(np.expand_dims(v,axis=0))))
    S = np.vstack((S,np.array([[0,0,0,0]])))
    # t = np.dot(np.identity(3)*theta + (1-math.cos(theta))*w_hat +
    #           (theta-math.sin(theta))*np.dot(w_hat,np.transpose(w_hat)),v)
    # R = expm(w_hat*theta)
    # output = np.hstack((R,np.expand_dims(t,axis=1)))
    # output = np.vstack((output,np.array([[0,0,0,1]])))
    output = expm(S*theta)
    return output



def FK_pox(joint_angles, m_mat, s_lst):
    """!
    @brief      Get a  representing the pose of the desired link

                TODO: implement this function, Calculate forward kinematics for rexarm using product of exponential
                formulation return a 4x4 homogeneous matrix representing the pose of the desired link

    @param      joint_angles  The joint angles
                m_mat         The M matrix
                s_lst         List of screw vectors

    @return     a 4x4 homogeneous matrix representing the pose of the desired link
    """
    product1 = rodrigues(joint_angles[0],s_lst[0][0:3],s_lst[0][3:6])
    product2 = rodrigues(-1*joint_angles[1],s_lst[1][0:3],s_lst[1][3:6])
    product3 = rodrigues(joint_angles[2],s_lst[2][0:3],s_lst[2][3:6])
    product4 = rodrigues(joint_angles[3],s_lst[3][0:3],s_lst[3][3:6])
    product5 = rodrigues(joint_angles[4],s_lst[4][0:3],s_lst[4][3:6])

    product = np.dot(product1,np.dot(product2,np.dot(product3,np.dot(product4,np.dot(product5,m_mat)))))

    return product

def to_s_matrix(w, v):
    """!
    @brief      Convert to s matrix.

    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)

    @param      w     { parameter_description }
    @param      v     { parameter_description }

    @return     { description_of_the_return_value }
    """
    s_matrix = np.hstack((w,v))
    pass


def IK_geometric(pose):
    """!
    @brief      Get all possible joint configs that produce the pose.

                TODO: Convert a desired end-effector pose vector as np.array to joint angles

    @param      dh_params  The dh parameters
    @param      pose       The desired pose vector as np.array 

    @return     All four possible joint configurations in a numpy array 4x4 where each row is one possible joint
                configuration
    """
    x_c = pose[0,0]
    y_c = pose[1,0]
    z_c = pose[2,0]
    d_1 = 103.91 # [mm]
    l_2 = 205.73 # [mm]
    d_2 = 50 # [mm]
    l_3 = 200 # [mm]

    soln = np.zeros([4,5])

    # NOTE: all the following calculations are done without the link offsets, so we will need to correct these accordingly after calculating

    # first calculate the two possible --> pi/2 offset to account for difference in world frame versus IK frame
    # first solution for theta 1
    theta_11 = math.atan2(y_c,x_c) - math.pi/2
    # section solution is pi radians from the first solution
    theta_12 = math.pi/2 + math.atan2(y_c,x_c)

    # now calculating theta 3 , which will subsequently be used to calculate theta 2

        # note use -r when finding the theta 3 and theta 2 values with the second theta 1 solution
    
    r = math.sqrt( x_c**2 + y_c**2 )
    s = z_c - d_1 + 190
    # theta 3 calculation --> there are two solutions (elbow up and elbow down)
    # this is for the "normal" theta 1 configuration
    theta_31 = math.acos( ( ( r**2 + s**2) - l_2**2 - l_3**2 ) / ( 2*l_2*l_3 ) )
    theta_32 = -1 * math.acos( ( ( r**2 + s**2) - l_2**2 - l_3**2 ) / ( 2*l_2*l_3 ) )
    # theta 2 calculation corresponding to each theta 3 value
    theta_21 = math.atan2(s,r) - math.atan2( l_3*math.sin(theta_31), l_2+l_3*math.cos(theta_31))
    theta_22 = math.atan2(s,r) - math.atan2( l_3*math.sin(theta_32), l_2+l_3*math.cos(theta_32))

    # now lets calculate the other solutions for theta2 and theta3 but with the "abnormal" theta1 value

    theta_31_2 = math.acos( ( ( (-r)**2 + s**2) - l_2**2 - l_3**2 ) / ( 2*l_2*l_3 ) )
    theta_32_2 = -1 * math.acos( ( ( (-r)**2 + s**2) - l_2**2 - l_3**2 ) / ( 2*l_2*l_3 ) )
    
    theta_21_2 = math.atan2(s,-r) - math.atan2( l_3*math.sin(theta_31), l_2+l_3*math.cos(theta_31))
    theta_22_2 = math.atan2(s,-r) - math.atan2( l_3*math.sin(theta_32), l_2+l_3*math.cos(theta_32))
    
    # now we need to correct the theta_3 and theta_2 values to account for the offsets d_1 and the offset between the bicep and forearm links
    
    # first solution correction (theta1 = atan2(yc,xc) - pi.2)
    theta_31c = theta_31 + math.pi/2 - math.atan2(d_2,l_3)
    theta_32c = theta_32 + math.pi/2 - math.atan2(d_2,l_3)
    theta_21c = math.pi/2 - math.atan2(d_2,l_3) - theta_21
    theta_22c = math.pi/2 - math.atan2(d_2,l_3) - theta_22
    
    # section solution correction (theta1 = atan2(xc,yc) + pi.2)
    theta_31_2c =  theta_31_2 + math.pi/2 - math.atan2(d_2,l_3)
    theta_32_2c = theta_32_2 + math.pi/2 - math.atan2(d_2,l_3)
    theta_21_2c = math.pi/2 - math.atan2(d_2,l_3) - theta_21_2
    theta_22_2c = math.pi/2 - math.atan2(d_2,l_3) - theta_22_2

    # NOTE: SKIP THIS R03 SECTION I THINK FOR NOW IT IS NOT WORTH THE TROUBLE

    # calculating R03 now that theta 1-3 are known
    #th123 = np.zeros([4,3])
    #th123[0,:] = [theta_11 , theta_21c , theta_31c, theta_41 , theta_51]
    #th123[1,:] = [theta_11 , theta_22c , theta_32c, theta_42 , theta_52]
    #th123[2,:] = [theta_12 , theta_21_2c , theta_31_2c, theta_41_2 , theta_51_2]
    #th123[3,:] = [theta_12 , theta_22_2c , theta_32_2c, theta_41_2 , theta_51_2]
    
    #R03 = np.zeros([3,3])
    
    #for elem in th123:
        #R03

    # allotted space for calculating theta_4 and theta_5... rip haha

    #theta_41 = 0
    #theta_42 = 0
    #theta_51 = 0
    #theta_52 = 0
    
    #theta_41_2 = 0
    #theta_42_2 = 0
    #theta_51_2 = 0
    #theta_52_2 = 0

    # NOTE: NEW APPROACH FOR FINDING THETA_4 AND THETA_4 --> fix the EE to be pointing down, which lets us find theta_4.
    # Assume (until block detection is implemented) that the blocks edges are parallel to the grid. Thus when the end effector is pointing
    # down, theta_5 = -theta_1 to keep the end effector prongs aligned with the grid.

    psi = math.pi/2 # [degrees]

    alpha_1 = -math.pi + theta_21c - theta_31c
    alpha_2 = -math.pi + theta_22c - theta_32c
    alpha_3 = -math.pi + theta_21_2c - theta_31_2c
    alpha_4 = -math.pi + theta_22_2c - theta_32_2c

    theta_41 = alpha_1 + psi
    theta_42 = alpha_2 + psi
    theta_41_2 = alpha_3 + psi
    theta_42_2 = alpha_4 + psi

    theta_51 = theta_11
    theta_52 = theta_11
    theta_51_2 = theta_12
    theta_52_2 = theta_12

    soln[0,:] = [theta_11 , theta_21c , theta_31c, theta_41 , theta_51]
    soln[1,:] = [theta_11 , theta_22c , theta_32c, theta_42 , theta_52]
    soln[2,:] = [theta_12 , theta_21_2c , theta_31_2c, theta_41_2 , theta_51_2]
    soln[3,:] = [theta_12 , theta_22_2c , theta_32_2c, theta_42_2 , theta_52_2]

    return soln
