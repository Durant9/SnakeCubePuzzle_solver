'''
An easy and fast approach to solve the Snake Cube Puzzle.
The snake cube is treated as a robotic manipulator with 17 links and 16 rotational joints, each one with a possible joint angle of 0, pi/2, pi or 3pi/2.
The whole solution space is then composed of 4**16 possible combinations of joint angles. 
'''

from utils import *

l = [2, 2, 2, 2, 1, 1, 1, 2, 2, 1, 1, 2, 1, 2, 1, 1, 2]
valid_solutions = []
current_solution = '0'
can_explore_more = True

while can_explore_more:
    Q = str2angles(current_solution)
    _, ext_x = joints2points(Q, l)

    # If the current solution is valid, the exploration continues
    if is_valid_solution(ext_x):
        # If there are not all the joints in the solution, a 0 is appended
        if len(current_solution) < 16:
            current_solution += '0'
        # If all joints where already added, the solution is a valid one. Then the exploration continues updating the solution
        else:
            valid_solutions.append(current_solution)
            current_solution, can_explore_more = update_solution(current_solution, can_explore_more)
    
    # If the current solution is not valid, it gets updated
    else:
        current_solution, can_explore_more = update_solution(current_solution, can_explore_more)

# Saving the solution strings to a txt file
with open('solutions/solutions.txt', 'a') as f:
    f.truncate(0)
    f.write('\n'.join(valid_solutions))

# Saving the solution plots as images
for i, solution in enumerate(valid_solutions):
    Q = str2angles(solution)
    x, ext_x = joints2points(Q, l)
    plot_solution(x, ext_x, savedir = os.path.join('solutions', str(i) + '.jpg'))
