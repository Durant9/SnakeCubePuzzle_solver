'''
Tratto ogni giunto come un giunto rotoidale come in un manipolatore robotico. Dunque per ogni braccio trovo i parametri di
D-H in modo da calcolare tutte le posizioni dei giunti nello spazio.
Convenzione di D-H, considerando le terne di due giunti collegati da un braccio (X0, Y0, Z0) e (X1, Y1, Z1):
- Asse Z0 ed asse Z1 coincidenti con gli assi dei giunti 0 e 1 rispettivamente
- X0 si mette in direzione del giunto successivo, X1 corre lungo la normale comune tra Z0 e Z1
- Y0 e Y1 completano le terne
Parametri di D-H:
- d: distanza tra X0 e X1 lungo Z0
- theta: rotazione attorno a Z0 necessario per allneare X0 con X1
- a: distanza tra i centri delle terne lungo X1
- alpha: rotazione attorno a X1 per allineare Z0 con Z1
https://automaticaddison.com/how-to-find-denavit-hartenberg-parameter-tables/
'''

# ------------------------------     RICERCA SOLUZIONE     ------------------------------
# bruteforce non possibile (4^16 combinazioni) allora faccio backtracking con pruning. Partendo dal primo giunto si sceglie
# una rotazione dopo l'altra, poi appena si incontra una condizione che renderebbe invalida la soluzione (dx, dy, dz > 3 o interferenze)
# si smette di esplorare quella parte di albero. Tutte le soluzioni che arrivano ad una foglia sono soluzioni valide

from utils import *

l = [2, 2, 2, 2, 1, 1, 1, 2, 2, 1, 1, 2, 1, 2, 1, 1, 2]

# Lo spazio delle soluzioni è pensabile come un albero, dove ogni nodo è il valore di una variabile di giunto
# Se nell'esplorazione di un nodo la soluzione resta valida, si prosegue in profondità. Altrimenti, si torna al
# nodo genitore e si cambia il valore del nodo figlio

valid_solutions = []
current_solution = '0'
can_explore_more = True
while can_explore_more:
    Q = str2angles(current_solution)
    _, ext_x = joints2points(Q, l)

    # Se la soluzione corrente è ancora valida, esploro ulteriormente
    if is_valid_solution(ext_x):
        # Se non ho ancora messo tutti i giunti, aggiungo uno zero alla fine
        if len(current_solution) < 16:
            current_solution += '0'
        # Se l'ultimo giunto era il 16esimo, allora la soluzione corrente è una soluzione finale valida
        else:
            valid_solutions.append(current_solution)
            # Poi aggiorno la soluzione corrente incrementando l'ultimo angolo o tornando indietro
            current_solution, can_explore_more = update_solution(current_solution, can_explore_more)
    
    # Se la soluzione non è più valida
    else:
        current_solution, can_explore_more = update_solution(current_solution, can_explore_more)

print(valid_solutions)

# Configurazione 'standard':
# Q = np.array([-np.pi/2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -np.pi/2])

with open('solutions/solutions.txt', 'a') as f:
    f.truncate(0)
    f.write('\n'.join(valid_solutions))
# for i, solution in enumerate(valid_solutions):
#     Q = str2angles(solution)
#     x, ext_x = joints2points(Q, l)
#     plot_solution(x, ext_x, savedir = os.path.join('solutions', str(i) + '.jpg'))