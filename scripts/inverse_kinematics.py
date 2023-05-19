#!/usr/bin/env python
import rospy
import numpy as np
from sympy import cos as c
from sympy import sin as s
from numba import jit
from sympy import lambdify,nsimplify,simplify
from sympy import symbols,init_printing,Matrix,cos,sin
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
init_printing(use_unicode=True)

class RobotKinematics():
    def __init__(self):
        self.joint_states=symbols('theta_1', 'theta_2', 'theta_3', 'theta_4', 'theta_5', 'theta_6', 'theta_7')
        self.dh_params = Matrix([
                        [0, 0, 0, 0.0825, -0.0825, 0, 0.088],
                        [0, -np.pi / 2, np.pi / 2, np.pi / 2, -np.pi / 2, np.pi / 2, np.pi / 2],
                        [0.333, 0, 0.316, 0, 0.384, 0, 0.107],
                        [*self.joint_states]
                        ])
        self.limits = [
        (-2.8973, 2.8973),
        (-1.7628, 1.7628),
        (-2.8973, 2.8973),
        (-3.0718, -0.0698),
        (-2.8973, 2.8973),
        (-0.0175, 3.7525),
        (-2.8973, 2.8973)
        ]

    def update_joint_states(self):
        # TODO: define config file
        if True: current_joint_states = list(rospy.wait_for_message("/joint_states", JointState, rospy.Duration(10)).position)
        else: current_joint_states = list(rospy.wait_for_message("/franka_state_controller/joint_states_desired", JointState, rospy.Duration(10)).position)
        #for index in [2, 5, 9]: current_joint_states.insert(index, 0)
        self.dh_params[] = current_joint_states

    def calc_forward_kinematics(self):
        a_i, alpha_i, d_i, theta_i = symbols('a_i, alpha_i, d_i, theta_i')
        dh_matrix_i = Matrix([
            [c(theta_i), -s(theta_i), 0, a_i],
            [s(theta_i)*c(alpha_i), c(theta_i)*c(alpha_i), -s(alpha_i), -d_i*s(alpha_i)],
            [s(theta_i)*s(alpha_i), c(theta_i)*s(alpha_i), c(alpha_i), d_i*c(alpha_i)],
            [0, 0, 0, 1]
        ])

        dh_matrix = Matrix.eye(4)
        for i in [0, 4, 8, 12, 16, 20, 24]:
            dh_params_i = self.dh_params.T[i:i+4]
            dh_matrix = dh_matrix @ dh_matrix_i.subs({'a_i': dh_params_i[0], 'alpha_i': dh_params_i[1], 'd_i': dh_params_i[2], 'theta_i': dh_params_i[3]})

        dh_matrix = nsimplify(dh_matrix, tolerance=1e-10, rational=True)
        dh_matrix

    @jit 
    def calc_inverse_kinematics(self,step=0.1,max_iterations=1000,max_error=0.01):
        dh_matrix = dh_matrix[:3,:4].T
        flattened_matrix = np.ravel(dh_matrix).reshape(12, 1)
        jacobian=flattened_matrix.jacobian(Matrix(self.joint_states))
        jacobian = nsimplify(jacobian, tolerance=1e-10, rational=True)

        dh_matrix = jit(lambdify(self.joint_states, dh_matrix, 'numpy'), nopython=True)
        jacobian = jit(lambdify(self.joint_states, jacobian, 'numpy'), nopython=True)
        
        joint_states_init = np.array([l+(u-l)/2 for l, u in self.limits], dtype=np.float64).reshape(7, 1)
        dh_matrix_init = dh_matrix(*(joint_states_init).flatten())
        dh_matrix_goal = np.array([
            [1, 0, 0, 0.05],
            [0, 1, 0, 0.2],
            [0, 0, 1, 0.1],
            [0, 0, 0, 1],
        ])
        dh_matrix_goal = dh_matrix_goal[:3, :4].transpose().reshape(12, 1)
        delta=dh_matrix_goal-dh_matrix_init
        while np.max(np.abs(delta))>0.01:
            J_q=jacobian(*(joint_states_init).flatten())
            J_q=J_q/np.linalg.norm(J_q)
            delta_q=np.linalg.pinv(J_q) @ (delta*step)
            joint_states_init = joint_states_init + delta_q
            dh_matrix_init = dh_matrix(*(joint_states_init).flatten())
            delta = dh_matrix_goal - dh_matrix_init
            return joint_states_init     
    

def main():
    RK = RobotKinematics()

    publisher = rospy.Publisher('tf_own', Float64MultiArray, queue_size=1)
    rospy.init_node('inverse_kinematics', anonymous=True)
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        RK.update_joint_states()
        #dh_matrix_final = RK.calc_forward_kinematics()
        matrix_inverse = RK.calc_inverse_kinematics()
        
        msg = Float64MultiArray()
        msg.data = matrix_inverse

        rospy.loginfo(msg)
        publisher.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


