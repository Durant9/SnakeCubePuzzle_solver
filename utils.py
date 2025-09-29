from itertools import combinations
import numpy as np
from colorsys import hls_to_rgb
import matplotlib.pyplot as plt

def rainbow_color_stops(n=10, end=2/3):
    '''
    Creates colors uniformly from red to blue
    '''
    return [ hls_to_rgb(end * i/(n-1), 0.5, 1) for i in range(n) ]

def DH_computation(d, a, alpha, theta):
    '''
    Takes as input the joint's Denavit-Hartenberg parameters and returns homogeneous transformation matrix associated to that joint.
    '''
    T = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                 [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                 [0, np.sin(alpha), np.cos(alpha), d],
                 [0, 0, 0, 1]])
    return T

def is_valid_solution(x):
    '''
    Takes as input the list of all the cartesian points occupied by the manipulator and returns True if the solution is valid, False otherwise.
    '''
    adj_x = np.array([np.array([round(x_[0]), round(x_[1]), round(x_[2])]) for x_ in x])
    dx = max(adj_x[:, 0]) - min(adj_x[:, 0])
    dy = max(adj_x[:, 1]) - min(adj_x[:, 1])
    dz = max(adj_x[:, 2]) - min(adj_x[:, 2])

    interference = any([all(x1 == x2) for x1, x2 in combinations(adj_x, 2)])

    if interference or dx > 2 or dy > 2 or dz > 2:
        return False
    else:
        return True

def joints2points(Q, all_l):
    '''
    Takes as input the joint angles vector and the links' lenghts and returns the cartesian points occupied by the manipulator in that configuration.
    '''
    # DH parameters
    n_joints = len(Q)
    if n_joints <= 14: # 2(+2)-15(+2) terne totali --> 3-16 parametri DH. +2 = bisogna trovare centro terna 2 giunti dopo l'ultimo
        d = np.array([all_l[0], 0] + all_l[1:n_joints + 1] + [0])
        a = np.array([0] * (n_joints + 3))
        alpha = np.append(np.array([0]), np.pi / 2 * np.array([-1 if i % 2 == 0 else 1 for i in range(n_joints + 1)] + [0]))
        theta = np.append(np.insert(Q, 0, 0), np.array([0, 0]))
    elif n_joints == 15: # 16(+2) terne totali --> 17 parametri DH. +2 = centro terna ultimo giunto + centro terna effettore fittizio
        d = np.array([all_l[0], 0] + all_l[1:-1] + [0])
        a = np.array([0] * (n_joints + 3))
        alpha = np.append(np.array([0]), np.pi / 2 * np.array([-1 if i % 2 == 0 else 1 for i in range(n_joints)] + [0]*2))
        theta = np.append(np.insert(Q, 0, 0), np.array([0, 0]))
    elif n_joints == 16: # 18(+1) terne totali --> 18 parametri DH. +1 = cubo tra ultima terna ed effettore
        d = np.array([all_l[0]] + [0] + all_l[1:-1] + [0, 0])
        a = np.array([0] * 17 + [all_l[-1]] + [0])
        alpha = np.append(np.array([0]), np.pi / 2 * np.array([-1 if i % 2 == 0 else 1 for i in range(n_joints-1)] + [0]*3))
        theta = np.append(np.insert(Q, 0, 0), np.array([0, -np.pi / 2]))

    # Direct kinematics to find frames' centers
    x = [np.array([0,0,0])]
    cum_T = np.eye(4)
    for i, (d_, a_, alpha_, theta_) in enumerate(zip(d, a, alpha, theta)):
        new_T = DH_computation(d_, a_, alpha_, theta_)
        cum_T = np.matmul(cum_T, new_T)
        x.append(cum_T[0:3, 3])
    # Deleting overlapping frames' centers
    del x[2]
    del x[-1]

    # Extending the list of points to include intermediate ones in the links of length 2
    ext_x = [x[0]]
    for x_ in x[1:]:
        if np.linalg.norm(ext_x[-1] - x_) > 1:
            xm = (x_ + ext_x[-1]) / 2
            ext_x.append(xm)
        ext_x.append(x_)
        
    return x, ext_x

def plot_solution(x, ext_x, savedir = None):
    '''
    Takes as input the joint points and the total points occupied by the manipulator and plots it.
    '''
    colors = rainbow_color_stops(n=len(x) - 1)
    plt.figure(figsize = (12, 8))
    ax = plt.axes(projection='3d')
    ax.plot(x[0][0], x[0][1], x[0][2], 'or', markersize=8)
    for i in range(len(x) - 1):
        ax.plot([x[i][0], x[i + 1][0]], [x[i][1], x[i + 1][1]], [x[i][2], x[i + 1][2]], color=colors[i], linewidth=3)
    for x_ in ext_x[1:-1]:
        ax.plot(x_[0], x_[1], x_[2], 'ok', markersize=4)
    ax.plot(x[-1][0], x[-1][1], x[-1][2], 'ob', markersize=8)
    ax.set_xlim(max([int(x_[0]) for x_ in x]) + 1, min([int(x_[0]) for x_ in x]) - 1)
    ax.set_ylim(max([int(x_[1]) for x_ in x]) + 1, min([int(x_[1]) for x_ in x]) - 1)
    ax.set_zlim(max([int(x_[2]) for x_ in x]) + 1, min([int(x_[2]) for x_ in x]) - 1)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    plt.gca().invert_zaxis()
    plt.gca().invert_xaxis()
    plt.axis('equal')
    if savedir:
        plt.savefig(savedir)
    else:
        plt.show(block=True)

def str2angles(s):
    '''
    Takes as input the string with the digits and return the correspondent joint angles list.
    '''
    assert all([True if 4 > int(i) >= 0 else False for i in s]), "digits must be between 0 and 3"
    Q = []
    for i in s:
        Q.append(int(i) * np.pi/2)
    return Q

def update_solution(solution, can_explore_more):
    '''
    Takes as input the current solution and updates it following the backtracking-and-pruning algorythm. 
    '''
    # If the last angle has not beet completed yet, rotate it by pi/2
    if int(solution[-1]) < 3:
        solution = solution[:-1] + str(int(solution[-1]) + 1)

    # If it's been completed, backtrack to the deepest solution that does not have a '3' as last digit and rotate it by pi/2
    else:
        all_previous_nodes = [solution[:i] for i in range(1, len(solution))]
        valid_nodes = [s for s in all_previous_nodes if not s.endswith('3')]
        # If there are no more valid previous nodes, there's nothing more to explore and the algorythm ends
        if len(valid_nodes) == 0:
            can_explore_more = False
        else:
            solution = valid_nodes[-1]
            solution = solution[:-1] + str(int(solution[-1]) + 1)

    return solution, can_explore_more
