import sympy.core
from sympy import *
import numpy as np
import matplotlib.pyplot as plt

DEG_2_RAD = np.pi/180.0
RAD_2_DEG = 1/DEG_2_RAD


def get_angles(angle, prefix=''):
    if isinstance(angle, str):
        c = 'c' + prefix + str(angle)
        s = 's' + prefix + str(angle)
        cos_angle = symbols(c)
        sin_angle = symbols(s)
    elif isinstance(angle, sympy.Basic):
        if len(angle.args) > 1:
            if angle.args[0] == 90:
                cos_angle = sin(-angle.args[1])
                sin_angle = cos(angle.args[1])
            else:
                cos_angle = cos(angle)
                sin_angle = sin(angle)
        else:
            cos_angle = cos(angle)
            sin_angle = sin(angle)
    else:
        cos_angle = round(np.cos(angle*DEG_2_RAD), 2)
        sin_angle = round(np.sin(angle*DEG_2_RAD), 2)

    return cos_angle, sin_angle


def get_val(val):
    if isinstance(val, str):
        return symbols(val)
    else:
        return val


def get_transformation_matrix(dh_parameters):
    # get values from dh params list
    a = get_val(dh_parameters[0])
    alpha = dh_parameters[1]
    d = get_val(dh_parameters[2])
    theta = dh_parameters[3]

    # assign symbols to cos and sin angles
    cos_alpha, sin_alpha = get_angles(alpha, 'a')
    cos_theta, sin_theta = get_angles(theta)

    """
    Ai = Rot(z, θi).Trans(z, di).Trans(x, ai).Rot(x,αi)
    """
    # get transformation matrix Ai
    T_i = Matrix([[cos_theta, -sin_theta*cos_alpha, sin_theta*sin_alpha, a*cos_theta],
                  [sin_theta, cos_theta*cos_alpha, -cos_theta*sin_alpha, a*sin_theta],
                  [0, sin_alpha, cos_alpha, d],
                  [0, 0, 0, 1]])
    return T_i.evalf(1)


def get_final_matrix(dh_parameters, params_idx=None):
    identity_matrix = Matrix([[1, 0, 0, 0],
                              [0, 1, 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])
    final_matrix = identity_matrix
    if params_idx is None:
        for params in dh_parameters:
            final_matrix *= get_transformation_matrix(params)
    else:
        for idx in range(params_idx+1):
            final_matrix *= get_transformation_matrix(dh_parameters[idx])

    return final_matrix.evalf(1)

def generate_workspace_points(num_points=100):

    t1_angles = np.linspace(0, 360, num_points)
    t2_angles = np.linspace(-90, 90, num_points)
    t3_angles = np.linspace(-90, 90, num_points)
    t4_angles = np.linspace(0, 360, num_points)
    t5_angles = np.linspace(-90, 90, num_points)
    t6_angles = np.linspace(0, 360, num_points)

    D1, A2, D3, D4, A5 = 1, 1, 1, 1, 1

    X = []
    Y = []
    Z = []

    points = 0
    for t1 in t1_angles:
        for t2 in t2_angles:
            for t3 in t3_angles:
                for t4 in t4_angles:
                    for t5 in t5_angles:
                        for t6 in t6_angles:
                            points += 1
                            if points % 100 == 0:
                                print("points generated: ", points)
                                dh_params = [[0, -90, D1, t1],
                                             [-A2, 0, 0, t2 + 90],
                                             [0, 90, 0, t3 - 90],
                                             [0, -90, D3 + D4, t4],
                                             [A5, 0, 0, t5 - 90],
                                             [0, 0, 0, t6]]

                                final_transformation = get_final_matrix(dh_params)

                                end_effector_x = final_transformation.col(-1)[0]
                                end_effector_y = final_transformation.col(-1)[1]
                                end_effector_z = final_transformation.col(-1)[2]

                                X.append(end_effector_x)
                                Y.append(end_effector_y)
                                Z.append(end_effector_z)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(X, Y, Z)
    plt.show()


if __name__ == "__main__":

    generate_workspace_points(5)




