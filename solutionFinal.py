# Look for #IMPLEMENT tags in this file. These tags indicate what has
# to be implemented to complete the Sokoban warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

# import os for time functions
from search import *  # for search engines
from sokoban import SokobanState, Direction, PROBLEMS, sokoban_goal_state  # for Sokoban specific classes and problems


# SOKOBAN HEURISTICS
def heur_displaced(state):
    '''trivial admissible sokoban heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    count = 0
    for box in state.boxes:
        if box not in state.storage:
            count += 1
        return count


def heur_manhattan_distance(state):
    # IMPLEMENT
    '''admissible sokoban heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # We want an admissible heuristic, which is an optimistic heuristic.
    # It must always underestimate the cost to get from the current state to the goal.
    # The sum Manhattan distance of the boxes to their closest storage spaces is such a heuristic.
    # When calculating distances, assume there are no obstacles on the grid and that several boxes can fit in one storage bin.
    # You should implement this heuristic function exactly, even if it is tempting to improve it.
    # Your function should return a numeric value; this is the estimate of the distance to the goal.
    total_man_dist = 0
    for box in state.boxes:
        man_dist = float('inf')
        for storage in state.storage:
            man_dist = min(man_dist, abs(box[0] - storage[0]) + abs(box[1] - storage[1]))
        total_man_dist += man_dist
    return total_man_dist


def heur_alternate(state):
    # IMPLEMENT
    '''a better sokoban heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # Rationale for alt. heuristics:
    # Step 1: Identify boxes on already at a storage spot (they could still be moved, but it's not accounted for in this heuristic
    # Step 2: Encourage steps that push a box onto a storage and penalize steps that move boxes from storage
    # Step 3: Check for deadlock situation:
    #   3.1: At least two of the four adjacent spots of a box is either an obstacle or wall
    #       (e.g. wall on the left, obstacle on the top) except the case when the box is between two obstacles.
    #   3.2: Two boxes on the boundary
    # Step 4: Calculate actual distance from each box to each storage obstacles
    #   4.1: For a box at (xb,yb) and a stroage at (xs,ys), consider the rectangle (xb:xs, yb:ys). If there are obstacles
    #       that covers every single row or col within the rectangle, the increase the distance by two. If the rectangle is blocked,
    #       extend the rectangle by 2 units in the direction of blockage and check for blockage until both walls are reached.
    #   4.2: If a box is on the boundary, any storages that are not on that boundary will be assigned to a distance of 100 since it will never reach
    # Step 5: Use Hungarian algorithm to find the minimum total distance from all the misplaced boxes to the closest available storage
    # However, if there is only one box left, simply find the min dist from that box to all the storages left
    # Step 6: Given the box-storage pairing in step 4, find the position(s) that the robot sohuld be in order to push boxes towards the storage
    #       (e.g. if storage is (0,0) and box is (3,0), then robot should go to (4,0) instead of (2,0) even though both spots are beside the box)
    # Step 7: Assign robots to misplaced boxes
    # Step 8: The heuristic is the total distance from assigned robots to each misplaced box plus the total distance from each misplaced box to its assigned storage plus penalty

    # Compare # of correctly placed boxes with its parent
    repeats = set(state.boxes & state.storage)
    penalty = 0
    parent = state.parent
    if parent is not None:
        repeats_parent = set(state.parent.boxes & state.parent.storage)
        if len(repeats) > len(repeats_parent):
            return 0
        elif len(repeats) < len(repeats_parent):
            penalty += 5

    # check for deadlocks
    deadlock_heuristic = 1000
    for box in state.boxes:
        if box not in repeats:  # if box is not already on a storage spot
            # 2 scenarios:
            # 1: box is adjacent to 2 obstacles or wall (not across from each other)
            if ((((box[0] - 1 < 0) | ((box[0] - 1, box[1]) in state.obstacles)) & (
                (box[1] - 1 < 0) | ((box[0], box[1] - 1) in state.obstacles))) |
                    (((box[0] + 1 >= state.width) | ((box[0] + 1, box[1]) in state.obstacles)) & (
                        (box[1] - 1 < 0) | ((box[0], box[1] - 1) in state.obstacles))) |
                    (((box[0] + 1 >= state.width) | ((box[0] + 1, box[1]) in state.obstacles)) & (
                        (box[1] + 1 >= state.height) | ((box[0], box[1] + 1) in state.obstacles))) |
                    (((box[0] - 1 < 0) | ((box[0] - 1, box[1]) in state.obstacles)) & (
                        (box[1] + 1 >= state.height) | ((box[0], box[1] + 1) in state.obstacles)))):
                return deadlock_heuristic

    # copy boxes and storages from frozenset to list
    boxes = []
    storages = []
    for box in state.boxes:
        boxes.append(box)
    for storage in state.storage:
        storages.append(storage)

    # Avoid two adjacent boxes on the boundary unless they're both on a storage spot
    boundary_boxes = [(a, b) for (a, b) in boxes if a == 0]  # boundary x = 0
    num_bound_boxes = len(boundary_boxes)
    if num_bound_boxes > 1:  # at least 2 boxes on the bounary
        for i in range(num_bound_boxes):
            for j in range(i + 1, num_bound_boxes):
                if abs(boundary_boxes[i][1] - boundary_boxes[j][1]) == 1:  # if adjacent
                    if (boundary_boxes[i] not in storages) | (
                        boundary_boxes[j] not in storages):  # if not both boxes on at a storage spot
                        return deadlock_heuristic
    boundary_boxes = [(a, b) for (a, b) in boxes if a == state.width - 1]  # boundary x = width-1
    num_bound_boxes = len(boundary_boxes)
    if num_bound_boxes > 1:
        for i in range(num_bound_boxes):
            for j in range(i + 1, num_bound_boxes):
                if abs(boundary_boxes[i][1] - boundary_boxes[j][1]) == 1:  # if adjacent
                    if (boundary_boxes[i] not in storages) | (boundary_boxes[j] not in storages):
                        return deadlock_heuristic
    boundary_boxes = [(a, b) for (a, b) in boxes if b == 0]  # boundary y = 0
    num_bound_boxes = len(boundary_boxes)
    if num_bound_boxes > 1:
        for i in range(num_bound_boxes):
            for j in range(i + 1, num_bound_boxes):
                if abs(boundary_boxes[i][0] - boundary_boxes[j][0]) == 1:
                    if (boundary_boxes[i] not in storages) | (boundary_boxes[j] not in storages):
                        return deadlock_heuristic
    boundary_boxes = [(a, b) for (a, b) in boxes if b == state.height - 1]  # boundary y = height -1
    num_bound_boxes = len(boundary_boxes)
    if num_bound_boxes > 1:
        for i in range(num_bound_boxes):
            for j in range(i + 1, num_bound_boxes):
                if abs(boundary_boxes[i][0] - boundary_boxes[j][0]) == 1:
                    if (boundary_boxes[i] not in storages) | (boundary_boxes[j] not in storages):
                        return deadlock_heuristic

    # remove boxes (and storages) that are already at a storage space
    for box in state.boxes:
        if box in repeats:
            boxes.remove(box)
            storages.remove(box)
    if (not boxes) | (not storages):
        return 0

    # calculate min dist from box to storages
    push_pos = []
    # If 1 box left to fill, assign to closest storage
    if len(boxes) == 1:  # (len(boxes) - len(repeats)) == 1:#
        man_dist = float('inf')
        box = box in boxes and box not in storages
        for i in range(len(storages)):
            temp_man_dist = calculate_man_dist(boxes[0], storages[i], state.obstacles, state.width, state.height, True)
            if temp_man_dist < man_dist:
                man_dist = temp_man_dist
                min_ind = i
        pos = calc_push_pos(boxes[0], storages[i], state.obstacles, boxes, state.width, state.height)
        push_pos.append(pos)
        # print('1 box')
    else:
        # more than 1 box left
        man_dist_matrix_original = [[0] * len(storages) for i in range(len(boxes))]
        for i in range(len(boxes)):
            for j in range(len(storages)):
                cur_man_dist = calculate_man_dist(boxes[i], storages[j], state.obstacles, state.width, state.height,
                                                  True)
                man_dist_matrix_original[i][j] = cur_man_dist
        final_zeros_star = find_min_cost(man_dist_matrix_original)
        # add up all entries in man_dist_matrix_original in which M contains zero*
        man_dist = 0
        for zero in final_zeros_star:
            man_dist += man_dist_matrix_original[zero[0]][zero[1]]

        # assign ideal position where robot should push the boxes from (or total number of available spots to push)
        for zero in final_zeros_star:
            box = boxes[zero[0]]
            storage = storages[zero[1]]
            pos = calc_push_pos(box, storage, state.obstacles, boxes, state.width, state.height)
            push_pos.append(pos)

    # drive robots to boxes
    # 1 robot
    if len(state.robots) == 1:
        robot = state.robots[0]
        robot_man_dist = float('inf')
        for pos in push_pos:  # only consider boxes that are not at a storage yet
            if (pos[0] < 0) | (pos[1] < 0) | (pos[0] >= state.width) | (pos[1] >= state.height):
                a = 1
            else:
                temp_dist = calculate_man_dist(robot, pos, state.obstacles,
                                               state.width, state.height, False)
                robot_man_dist = min(robot_man_dist, temp_dist)
    else:  # if more than 1 robot
        if len(push_pos) > len(state.robots):
            robot_man_dist_matrix = [[0] * len(push_pos) for i in range(len(state.robots))]
            for i in range(len(state.robots)):
                for j in range(len(push_pos)):
                    cur_man_dist = calculate_man_dist(state.robots[i], push_pos[j], state.obstacles,
                                                      state.width, state.height, False)
                    robot_man_dist_matrix[i][j] = cur_man_dist
            final_zeros_star = find_min_cost(robot_man_dist_matrix)
        else:
            robot_man_dist_matrix = [[0] * len(state.robots) for i in range(len(push_pos))]
            for i in range(len(push_pos)):
                for j in range(len(state.robots)):
                    cur_man_dist = calculate_man_dist(push_pos[i], state.robots[j], state.obstacles,
                                                      state.width, state.height, False)
                    robot_man_dist_matrix[i][j] = cur_man_dist
            final_zeros_star = find_min_cost(robot_man_dist_matrix)
        # add up all entries in man_dist_matrix_original in which M contains zero*
        robot_man_dist = 0
        for zero in final_zeros_star:
            robot_man_dist += robot_man_dist_matrix[zero[0]][zero[1]]

    if (len(boxes) == 1) & (man_dist < deadlock_heuristic) : #if one misplaced box remaining
        #encourage robots to move towards boxes
        return robot_man_dist + man_dist / 2.0 + penalty
    else:
        return man_dist + robot_man_dist + penalty
#reward for pushing min box
################################Helper functions for alternate_heuristic

# calculate position where robot should push the box
def calc_push_pos(box, storage, obstacles, boxes, width, height):
    if box[0] == storage[0]:  # same x
        if box[1] > storage[1]:
            return ((box[0], box[1] + 1))
        else:
            return ((box[0], box[1] - 1))
    elif box[1] == storage[1]:  # same y
        if box[0] > storage[0]:
            return ((box[0] + 1, box[1]))
        else:
            return ((box[0] - 1, box[1]))
    else:  # can place at two positions (push horizontal first or vertical first), pick the one that will not run into obstacles
        if (box[0] > storage[0]) & (box[1] > storage[1]):
            pos1 = [(box[0], box[1] + 1),
                    (box[0], box[1] - 1)]  # position to push and position where the box will be pushed to
            pos2 = [(box[0] + 1, box[1]), (box[0] - 1, box[1])]
        elif (box[0] > storage[0]) & (box[1] < storage[1]):
            pos1 = [(box[0], box[1] - 1), (box[0], box[1] + 1)]
            pos2 = [(box[0] + 1, box[1]), (box[0] - 1, box[1])]
        elif (box[0] < storage[0]) & (box[1] < storage[1]):
            pos1 = [(box[0], box[1] - 1), (box[0], box[1] + 1)]
            pos2 = [(box[0] - 1, box[1]), (box[0] + 1, box[1])]
        else:
            pos1 = [(box[0], box[1] + 1), (box[0], box[1] - 1)]
            pos2 = [(box[0] - 1, box[1]), (box[0] + 1, box[1])]

        # -1: no deadlock, 0: obs or wall, 1: boxes
        box1_check = deadlock_check(pos1, obstacles, boxes, -1, height)
        box2_check = deadlock_check(pos2, obstacles, boxes, width, -1)
        if box1_check >= 0:
            if box2_check >= 0:
                # when both boxes are blocked
                if box1_check == 0:
                    return pos2[0]
                else:
                    return pos1[0]
            else:
                return (pos2[0])
        else:
            return (pos1[0])

# deadlock helper function (to check x only, set height to -1, to check y only set width to -1)
def deadlock_check(points, obstacles, boxes, width, height):
    if width > 0:
        for point in points:
            if (point[0] < 0) | (point[0] >= width) | (point in obstacles):  # deadlock found
                return 0  # obs found
            if (point in boxes):
                return 1  # box found
    if height > 0:
        for point in points:
            if (point[1] < 0) | (point[1] >= height) | (point in obstacles):  # deadlock found
                return 0
            if (point in boxes):
                return 1
    return -1

# calculate true manhatten distance accounting for obstacles
def calculate_man_dist(box, storage, obstacles, width, height, check_boundary):
    raw_man_dist = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
    # box on a storage
    if raw_man_dist == 0:
        return 0
    # box at the boundary but not the storage (cannot possibly reach)
    if check_boundary:
        if ((box[0] == 0) & (storage[0] != 0)) | ((box[0] == width - 1) & (storage[0] != width - 1)) | (
            (box[1] == 0) & (storage[1] != 0)) | ((box[1] == height - 1) & (storage[1] != height - 1)):
            return 1000
    if not obstacles:
        return raw_man_dist
    else:
        offsetX = abs(box[0] - storage[0])
        offsetY = abs(box[1] - storage[1])
        minX = min(box[0], storage[0])
        maxX = max(box[0], storage[0])
        minY = min(box[1], storage[1])
        maxY = max(box[1], storage[1])
        if offsetX >= 2:
            nearby_obs = []
            for obstacle in obstacles:
                range_low = minY
                range_high = maxY
                if (obstacle[0] in range(minX + 1, maxX)) & (obstacle[1] in range(range_low, range_high + 1)):
                    nearby_obs.append(obstacle)
            while (height >= range_high - range_low) & (
                        len(set([x[1] for x in nearby_obs])) == range_high - range_low + 1):
                range_low = max(range_low - 1, 0)
                range_high = min(range_high + 1, height - 1)
                raw_man_dist = raw_man_dist + 2
                for obstacle in obstacles:
                    if (obstacle[0] in range(minX + 1, maxX)) & (obstacle[1] in [range_low, range_high]):
                        nearby_obs.append(obstacle)
        if offsetY >= 2:
            nearby_obs = []
            for obstacle in obstacles:
                range_low = minX
                range_high = maxX
                if (obstacle[0] in range(range_low, range_high + 1)) & (obstacle[1] in range(minY + 1, maxY)):
                    nearby_obs.append(obstacle)
            while (width >= range_high - range_low) & (
                        len(set([x[0] for x in nearby_obs])) == range_high - range_low + 1):
                range_low = max(range_low - 1, 0)
                range_high = min(range_high + 1, width - 1)
                raw_man_dist = raw_man_dist + 2
                for obstacle in obstacles:
                    if (obstacle[0] in [range_low, range_high]) & (obstacle[1] in range(minY + 1, maxY)):
                        nearby_obs.append(obstacle)
    return raw_man_dist

#Hungarian algorithm:
# Algorithm works only if there are more (more equal) columns than rows, so assign each row as a box and each column as a storage
# man_dist_matrix: col: storages, row: boxes
# M: 0: ordinary 0's in man_dist_matrix, 1: starred zero, 2: primed zero
step = 0
row_covered = []
col_covered = []
num_row = 0
num_col = 0
M = []
man_dist_matrix = []
uncovered_zero_ind = (-1, -1)
final_zeros_star = []

def find_min_cost(man_dist_matrix_original):
    global M, row_covered, col_covered, num_row, num_col, man_dist_matrix, step, final_zeros_star
    man_dist_matrix = [row[:] for row in man_dist_matrix_original]
    # Step 1
    # for each row, find smallest element and subtract all elements in the same row by the min
    for i in range(len(man_dist_matrix)):
        min_val = min(man_dist_matrix[i])
        for j in range(len(man_dist_matrix[i])):
            man_dist_matrix[i][j] -= min_val

    # step 2
    # for each zero in the matrix, make it a zero* if there are no other zero*s in the same row or col
    num_row = len(man_dist_matrix)
    num_col = len(man_dist_matrix[0])
    M = [[-1] * num_col for i in range(num_row)]
    zeros = [(index, row.index(0)) for index, row in enumerate(man_dist_matrix) if 0 in row]
    for zero in zeros:
        if (zero[0] not in row_covered) & (zero[1] not in col_covered):
            M[zero[0]][zero[1]] = 1
            row_covered.append(zero[0])
            col_covered.append(zero[1])
        else:
            M[zero[0]][zero[1]] = 0
    # reset row and col covers
    row_covered = []
    col_covered = []
    step = 3
    runMunkres()
    return final_zeros_star


def runMunkres():
    global step, man_dist_matrix, M
    printStep = False
    while (1):
        if step == 3:
            step3()
        elif step == 4:
            step4()
        elif step == 5:
            step5()
        elif step == 6:
            step6()
        elif step == 7:
            return

def step3():
    global col_covered, num_col, final_zeros_star, step
    # cover all columns that contain zero*
    zeros_star = [(index, row.index(1)) for index, row in enumerate(M) if 1 in row]
    if not zeros_star:
        col_covered = []
    else:
        unique_col = set([int(i[1]) for i in zeros_star])
        col_covered = [i for i in unique_col]
    # if each col contains a zero* the solution is found
    if (len(col_covered) >= num_col) | (len(col_covered) >= num_row):
        final_zeros_star = zeros_star
        step = 7
    else:
        step = 4
    return

def step4():
    global M, row_covered, col_covered, man_dist_matrix, uncovered_zero_ind, step
    done = False
    while ~done:
        # find a non-covered zero
        uncovered_zero_found = False
        exit = False
        for i in range(len(M)):
            for j in range(len(M[0])):
                if (man_dist_matrix[i][j] == 0) & (not i in row_covered) & (not j in col_covered):
                    uncovered_zero_found = True
                    M[i][j] = 2
                    if 1 not in M[i]:  # no starred zero in the row containing the zero that was just found
                        uncovered_zero_ind = (i, j)
                        step = 5
                        return
                    else:
                        row_covered.append(i)
                        col_covered.remove(M[i].index(1))
                        exit = True
                        break
            if exit:
                break
        if not uncovered_zero_found:
            done = True
            step = 6
            return

def step5():  # i,j are the row and col of the uncovered zero found in step 4 (primed in step 4)
    global row_covered, col_covered, M, uncovered_zero_ind, step
    i = uncovered_zero_ind[0]
    j = uncovered_zero_ind[1]
    alt_path = [(i,
                 j)]  # path root: (row, col, 0 or 1) where 0 mean next entry should look for rows and 1 means next entry should look for cols
    count = 1
    while (1):
        # Let z0 be the uncovered zero in step 4, find a starred zero in the same column
        col = alt_path[-1][1]
        if 1 not in [row[col] for row in M]:
            break
        else:
            row = [row[col] for row in M].index(1)  # z[-1][1]: col of the last node in z (index 0: row, 1: col)
            count += 1
            alt_path.append((row, alt_path[-1][1]))
            # find prime in row (should always be one)
            col = M[row].index(2)
            count += 1
            alt_path.append((alt_path[-1][0], col))
    # augment_path
    for node in alt_path:
        if M[node[0]][node[1]] == 2:
            M[node[0]][node[1]] = 1
        elif M[node[0]][node[1]] == 1:
            M[node[0]][node[1]] = 0
    # clear covers
    row_covered = []
    col_covered = []
    # erase primes
    for i in range(len(M)):
        for j in range(len(M[i])):
            if M[i][j] == 2:
                M[i][j] = 0
    step = 3
    return


def step6():
    global row_covered, col_covered, man_dist_matrix, M, num_col, step
    # find min val
    min_val = float('inf')
    for i in range(len(man_dist_matrix)):
        for j in range(len(man_dist_matrix[i])):
            if (i not in row_covered) & (j not in col_covered):
                min_val = min(min_val, man_dist_matrix[i][j])

    for i in range(len(man_dist_matrix)):
        for j in range(len(man_dist_matrix[i])):
            if ((i in row_covered) & (j not in col_covered)) | ((i not in row_covered) & (j in col_covered)):
                continue
            elif i in row_covered:
                man_dist_matrix[i][j] += min_val
                M[i][j] = -1  # new dist value will always be greater than 0
            elif j not in col_covered:
                man_dist_matrix[i][j] -= min_val
                if man_dist_matrix[i][j] == 0:
                    M[i][j] = 0
                    # elif (man_dist_matrix[i][j] > 0) & (M[i][j] >= 0):
                    #    M[i][j] = -1 #might not need
    step = 4
    return

###############################End of helper functions for heuristic_alternate

def fval_function(sN, weight):
    # IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """

    # Many searches will explore nodes (or states) that are ordered by their f-value.
    # For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    # You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    # The function must return a numeric f-value.
    # The value will determine your state's position on the Frontier list during a 'custom' search.
    # You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.


    return (1 - weight) * sN.gval + weight * sN.hval


def weighted_astar(initial_state, timebound):
    # IMPLEMENT
    '''Provides an implementation of weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''

    se = SearchEngine('custom', 'full')  # custom setting uses fval
    weight = 1.0  # starts at 1 and decrease by 0.1 every iteration
    total_time = 0
    count = 0
    min_cost = float('inf')
    while total_time < timebound:
        final = se.search(initState=initial_state, heur_fn=heur_alternate, timebound=timebound - total_time,
                          goal_fn=sokoban_goal_state, fval_function=fval_function, weight=weight)
        if final:
            total_time += se.total_search_time
            if final.gval <= min_cost:
                min_cost = final.gval  # update min_cost
                last_success = final  # save state with min_cost
            if weight == 0:
                return last_success  # if weight = 0, f_val = g_val and the solution must be optimal
            else:
                weight -= 0.1
                count += 1  # keep track of # of times a solution was found
        else:
            if count > 0:
                return last_success
            else:
                return final  # no solution found, return failed result