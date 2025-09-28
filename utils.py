from itertools import combinations
import numpy as np
from colorsys import hls_to_rgb
import matplotlib.pyplot as plt

def rainbow_color_stops(n=10, end=2/3):
    return [ hls_to_rgb(end * i/(n-1), 0.5, 1) for i in range(n) ]

def DH_computation(d, a, alpha, theta):
    '''
    prende in ingresso i parametri di Denavit-Hartemberg del giunto e restituisce
    la matrice di trasformazione omogenea associata a quel giunto
    '''
    T = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                 [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                 [0, np.sin(alpha), np.cos(alpha), d],
                 [0, 0, 0, 1]])
    return T

def is_valid_solution(x):
    '''
    Prende tutti i punti della soluzione, restituisce True se è valida, False altrimenti
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
    Prende il vettore delle variabili di giunto e le lunghezze dei bracci e restituisce i punti occupati dal manipolatore.
    SEGUENDO LO SCHEMA CC3, SE CI SONO TUTTI I GIUNTI (N=16):
    - d = [l[0], 0, l[1:-1]] (il primo, uno 0, poi tutti gli altri escluso l'ultimo)
    - a = [0]*16 + [l[-1]] (16 zeri, poi la lunghezza dell'ultimo link, provare con e senza meno)
    - alpha = [0, -pi/2, pi/2, -pi/2, ..., 0] (uno 0, poi 15 pi/2 a segno alternato partendo da -, poi un altro 0)
    - theta = Q.copy() + [-pi/2]
    Se ci sono meno di N-1 giunti attivi, allora l'EE è il centro della terna 2 giunti dopo l'ultimo (es. se ci sono
    8 giunti attivi l'EE si trova al centro della terna 10, con theta_9 = 0)
    Se ci sono N-1 giunti attivi, l'EE si trova aggiungendo una terna fittizia con d=1 e gli altri parametri a 0
    '''
    # Parametri di DH
    n_joints = len(Q)
    if n_joints <= 14: # 2(+2)-15(+2) terne totali --> 3-16 parametri DH. +2 = bisogna trovare centro terna 2 giunti dopo l'ultimo
        d = np.array([all_l[0], 0] + all_l[1:n_joints + 1] + [0])
        a = np.array([0] * (n_joints + 3))
        alpha = np.append(np.array([0]), np.pi / 2 * np.array([-1 if i % 2 == 0 else 1 for i in range(n_joints + 1)] + [0]))
        theta = np.append(np.insert(Q, 0, 0), np.array([0, 0]))
        # d = np.array([all_l[0], 0] + all_l[1:n_joints+1])
        # a = np.array([0]*(n_joints+2))
        # alpha = np.append(np.array([0]), np.pi / 2 * np.array([-1 if i % 2 == 0 else 1 for i in range(n_joints+1)]))
        # theta = np.append(Q.copy(), np.array([0, 0]))
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

    # Cinematica diretta per trovare le posizioni dei giunti
    x = [np.array([0,0,0])]
    cum_T = np.eye(4)
    for i, (d_, a_, alpha_, theta_) in enumerate(zip(d, a, alpha, theta)):
        new_T = DH_computation(d_, a_, alpha_, theta_)
        cum_T = np.matmul(cum_T, new_T)
        x.append(cum_T[0:3, 3])
    # Eliminazione del centro del secondo giunto, sovrapposto col primo
    del x[2]
    del x[-1]

    # Estensione di x per comprendere anche i punti intermedi
    ext_x = [x[0]]
    for x_ in x[1:]:
        if np.linalg.norm(ext_x[-1] - x_) > 1:
            xm = (x_ + ext_x[-1]) / 2
            ext_x.append(xm)
        ext_x.append(x_)
    return x, ext_x

def plot_solution(x, ext_x, savedir = None):
    '''
    Prende i punti della soluzione e plotta il manipolatore.
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
    assert all([True if 4 > int(i) >= 0 else False for i in s]), "codes must be between 0 and 3"
    Q = []
    for i in s:
        Q.append(int(i) * np.pi/2)
    return Q

def update_solution(solution, can_explore_more):
    # Se l'ultimo angolo non è ancora stato completato, lo ruoto
    if int(solution[-1]) < 3:
        solution = solution[:-1] + str(int(solution[-1]) + 1)

    # Se è stato completato, torno indietro alla più profonda soluzione che non ha come ultimo angolo 3 e cambio l'ultimo angolo
    else:
        all_previous_nodes = [solution[:i] for i in range(1, len(solution))]
        valid_nodes = [s for s in all_previous_nodes if not s.endswith('3')]
        # Se non ci sono soluzioni precedenti valide, allora non c'è più nulla da esplorare e l'algoritmo termina
        if len(valid_nodes) == 0:
            can_explore_more = False
        else:
            solution = valid_nodes[-1]
            solution = solution[:-1] + str(int(solution[-1]) + 1)
    return solution, can_explore_more