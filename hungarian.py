#algorithm works only if there are more (more equal) columns than rows, so assign each row as a box and each column as a storage
#col: box, row: storage
#M: 0: ordinary 0's in man_dist_matrix, 1: starred zero, 2: primed zero
step = 0
row_covered = []
col_covered = []
num_row = 0
num_col = 0
M = []
man_dist_matrix = []
#man_dist_matrix = [[1, 2, 3], [2, 4, 6], [3, 6, 9]]
#man_dist_matrix = [[8,1,9], [0,1,3], [6,5,7]]
#man_dist_matrix = [[3,7,0],[8,6,8],[7,8,3], [8,6,6]]
#man_dist_matrix = [[1,2,3,4,5,6,7,8,9,10],[2,4,6,8,10,12,14,16,18,20]]
#man_dist_matrix = [[3,1,8], [7,8,5], [7,8,3], [8,6,6]]
#man_dist_matrix = [[13,34,69],[50,77,34],[36,42,10], [40,71,64], [99,41,84]]
#man_dist_matrix = [[13,50,36,40,99],[34,77,42,71,41],[69,34,10,64,84]]

# num_row = 10
# num_col = 20
# man_dist_matrix = [x[:] for x in [[0] * num_col] * num_row]
# for i in range(num_row):
#     for j in range(num_col):
#         man_dist_matrix[i][j] = (i+1) * (j+1)

uncovered_zero_ind = (-1,-1)
final_zeros_star = []

def find_min_cost(man_dist_matrix_original):
    global M, row_covered, col_covered, num_row, num_col,man_dist_matrix, step, final_zeros_star
    man_dist_matrix = [row[:] for row in man_dist_matrix_original]
    #Step 1
    #for each row, find smallest element and subtract all elements in the same row by the min
    for i in range(len(man_dist_matrix)):
        min_val = min(man_dist_matrix[i])
        for j in range(len(man_dist_matrix[i])):
            man_dist_matrix[i][j] -= min_val


    #step 2
    #for each zero in the matrix, make it a zero* if there are no other zero*s in the same row or col
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
    #reset row and col covers
    row_covered = []
    col_covered = []
    step = 3
    runMunkres()
    # add up all entries in man_dist_matrix_original in which M contains zero*
    total_man_dist = 0
    for zero in final_zeros_star:
        total_man_dist += man_dist_matrix_original[zero[0]][zero[1]]
    return total_man_dist

def runMunkres():
    global step, man_dist_matrix, M
    printStep = False
    while(1):
        if step == 3:
            step3()
            if printStep:
                print('step3')
                print(man_dist_matrix)
                print(M)
        elif step == 4:
            step4()
            if printStep:
                print('step4')
                print(man_dist_matrix)
                print(M)
        elif step == 5:
            step5()
            if printStep:
                print('step5')
                print(man_dist_matrix)
                print(M)
        elif step == 6:
            step6()
            if printStep:
                print('step6')
                print(man_dist_matrix)
                print(M)
        elif step == 7:
            return

def step3():
    global col_covered, num_col, final_zeros_star, step
    #cover all columns that contain zero*
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
    global M,row_covered,col_covered, man_dist_matrix, uncovered_zero_ind, step
    done = False
    while ~done:
        #find a non-covered zero
        uncovered_zero_found = False
        exit = False
        for i in range(len(M)):
            for j in range(len(M[0])):
                if (man_dist_matrix[i][j] == 0) & (not i in row_covered) & (not j in col_covered):
                    uncovered_zero_found = True
                    M[i][j] = 2
                    if 1 not in M[i]:#no starred zero in the row containing the zero that was just found
                        uncovered_zero_ind = (i,j)
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


def step5():#i,j are the row and col of the uncovered zero found in step 4 (primed in step 4)
    global row_covered, col_covered, M, uncovered_zero_ind, step
    i = uncovered_zero_ind[0]
    j = uncovered_zero_ind[1]
    alt_path = [(i,j)] #path root: (row, col, 0 or 1) where 0 mean next entry should look for rows and 1 means next entry should look for cols
    count = 1
    while(1):
        #Let z0 be the uncovered zero in step 4, find a starred zero in the same column
        col = alt_path[-1][1]
        if 1 not in [row[col] for row in M]:
            break
        else:
            row = [row[col] for row in M].index(1) #z[-1][1]: col of the last node in z (index 0: row, 1: col)
            count += 1
            alt_path.append((row,alt_path[-1][1]))
            #find prime in row (should always be one)
            col = M[row].index(2)
            count +=1
            alt_path.append((alt_path[-1][0], col))
    #augment_path
    for node in alt_path:
        if M[node[0]][node[1]] == 2:
            M[node[0]][node[1]] = 1
        elif M[node[0]][node[1]] == 1:
            M[node[0]][node[1]] = 0
    #clear covers
    row_covered = []
    col_covered = []
    #erase primes
    for i in range(len(M)):
        for j in range(len(M[i])):
            if M[i][j] == 2:
                M[i][j] = 0
    step = 3
    return


def step6():
    global row_covered, col_covered, man_dist_matrix, M, num_col, step
    #find min val
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
                #for debugging
                if M[i][j] > 0:
                    a = 1
                man_dist_matrix[i][j] += min_val
                if man_dist_matrix[i][j] == 0:
                    M[i][j] = 0 #might not need
                else:
                    M[i][j] = -1
            elif j not in col_covered:
                if M[i][j] > 0: #for debugging (if star or prime zeros are changed)
                    a = 1
                man_dist_matrix[i][j] -= min_val
                if man_dist_matrix[i][j] < 0: #for debugging
                    a = 1
                if man_dist_matrix[i][j] == 0:
                    M[i][j] = 0
                else:
                    M[i][j] = -1 #might not need
    step = 4
    return

####################################################complete code for calculating robot and storage dist with seperate cases for obstacles and no obstacles
#drive robots to boxes
if not state.obstacles:#use simple man. dist.
    #1 robot
    if len(state.robots) == 1:
        robot = state.robots[0]
        robot_man_dist = float('inf')
        for box in boxes:#only consider boxes that are not at a storage yet
            robot_man_dist = min(robot_man_dist, abs(box[0] - robot[0]) + abs(box[1] - robot[1]))
    else:#if more than 1 robot
        robot_man_dist_matrix = [[0] * len(boxes) for i in range(len(state.robots))]
        for i in range(len(state.robots)):
            for j in range(len(boxes)):
                cur_man_dist = abs(state.robots[i][0]- boxes[j][0]) + abs(state.robots[i][1]- boxes[j][1])
                robot_man_dist_matrix[i][j] = cur_man_dist
        robot_man_dist = find_min_cost(robot_man_dist_matrix)
else:  # use calc. man. dist.
    # 1 robot
    if len(state.robots) == 1:
        robot = state.robots[0]
        robot_man_dist = float('inf')
        for box in boxes:  # only consider boxes that are not at a storage yet
            temp_dist = calculate_man_dist(robot, box, state.obstacles,
                               state.width, state.height)
            robot_man_dist = min(robot_man_dist, temp_dist)
    else:  # if more than 1 robot
        robot_man_dist_matrix = [[0] * len(boxes) for i in range(len(state.robots))]
        for i in range(len(state.robots)):
            for j in range(len(boxes)):
                cur_man_dist = calculate_man_dist(state.robots[i], boxes[j], state.obstacles,
                                                  state.width, state.height)
                robot_man_dist_matrix[i][j] = cur_man_dist
        robot_man_dist = find_min_cost(robot_man_dist_matrix)
robot_man_dist -= len(state.robots) #only need to move beside a box and not on top of it

# calculate min dist from box to storages
if not state.obstacles:
    # If 1 box left to fill, assign to closest storage
    if len(boxes) == 1:
        box = boxes[0]
        man_dist = float('inf')
        for storage in storages:
            man_dist = min(man_dist, abs(box[0] - storage[0]) + abs(box[1] - storage[1]))
    else:
        # more than 1 box left
        man_dist_matrix = [[0] * len(storages) for i in range(len(boxes))]
        for i in range(len(boxes)):
            for j in range(len(storages)):
                cur_man_dist = abs(boxes[i][0] - storages[j][0]) - abs(boxes[i][1] - storages[j][1])
                man_dist_matrix[i][j] = cur_man_dist
        man_dist = find_min_cost(man_dist_matrix)
else:
    # If 1 box left to fill, assign to closest storage
    if len(boxes) == 1:
        man_dist = float('inf')
        for storage in storages:
            man_dist = min(man_dist, calculate_man_dist(boxes[0], storage, state.obstacles, state.width, state.height))
    else:
        # more than 1 box left
        man_dist_matrix = [[0] * len(storages) for i in range(len(boxes))]
        for i in range(len(boxes)):
            for j in range(len(storages)):
                cur_man_dist = calculate_man_dist(boxes[i], storages[j], state.obstacles, state.width, state.height)
                man_dist_matrix[i][j] = cur_man_dist
        man_dist = find_min_cost(man_dist_matrix)

        # add both distances with a weight
return man_dist  # robot_man_dist