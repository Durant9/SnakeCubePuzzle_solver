'''
An easy and fast approach to solve the Snake Cube Puzzle.

THE PROBLEM
The snake cube is treated as a robotic manipulator with 17 links and 16 rotational joints, each one with a possible joint angle of 0, pi/2, pi or 3pi/2.
The whole solution space is then composed of 4**16 possible combinations of joint angles. A combination to be a valid solution needs to be such that the 
cartesian space occupied by the manipulator is a 3x3x3 cube without interferences between its parts. 

THE ALGORYTHM
It's impossible to explore the solution space with a greedy algorythm. A backtracking-and-pruning one is the used: the solution space can be seen as a big tree,
with each leaf being a combination of 16 joint angles. The nodes in the n-th layer represent a combination of the first n joint angles. The tree is explored from its root,
and as soon as an invalid node is found, the whole subtree under that node is pruned and will not be explored anymore. 
An invalid node is a node that represents a combination of joint angles such that the respective manipulator's configuration does not match the requirements needed to be a valid solution.

SOLUTION REPRESENTATION
The solution is represented by a string of up to 16 digits, each one going from 0 to 3. Each digit represents the rotation of a joint angle: '2130' stands for the joint angles [pi, pi/2, 3pi/2, 0]
'''

from utils import *
import os

# Initialization
# l --> lengths of the manipulator links
# valid_solutions --> list of the final valid solutions found
# current_solution --> string that represents the current solution to verify
# can_explore_more --> boolean that indicates the presence of unexplored nodes in the tree
l = [2, 2, 2, 2, 1, 1, 1, 2, 2, 1, 1, 2, 1, 2, 1, 1, 2] 
valid_solutions = []
current_solution = '0'
can_explore_more = True

# Backtracking-and-pruning algorythm
while can_explore_more:
    # Perform direct kinematics on the current solution to get the occupied points in cartesian space
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


