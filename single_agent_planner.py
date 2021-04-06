import heapq


def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)] # add time direction i.e. stay at the same place
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
                    or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that contains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    constraint_temp = []
    for x in constraints:
        if x['agent'] == agent:
            if (len(x['loc']) == 1):
                d3 = (x['timestep'], x['loc'][0])
            else:
                d3 = (x['timestep'], x['loc'][0], x['loc'][1])
            constraint_temp.append(d3)
    return constraint_temp


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
#        d3 = curr['loc'],curr['time']
#        path.append(d3)

        # find the tail location 
        orient = curr['orientation']
        head_loc = curr['loc']
        if orient == 1:
            tail_loc = (head_loc[0], head_loc[1] - 1)
        elif orient == 2:
            tail_loc = (head_loc[0] + 1, head_loc[1])
        elif orient == 3:
            tail_loc = (head_loc[0], head_loc[1] + 1)
        elif orient == 4:
            tail_loc = (head_loc[0] - 1, head_loc[1]) 
        else: # single cell 
            path.append(head_loc)
            curr = curr['parent']
            continue

        if curr['parent'] is None:

            path.append([head_loc,tail_loc])
        else: 
  
            parent_orient = curr['parent']['orientation']
            parent_loc = curr['parent']['orientation']
            # check if rotation in this time step 
            if orient != parent_orient:
                # if so append the intermediate tail location 
                if orient == 1:
                    if parent_orient == 2:
                        intermediate_loc = (tail_loc[0] + 1, tail_loc[1])
                    else: # 4
                        intermediate_loc = (tail_loc[0], tail_loc[1] - 1)
                elif orient == 2:
                    if parent_orient == 1:
                        intermediate_loc = (tail_loc[0], tail_loc[1] - 1)
                    else: # 3 
                        intermediate_loc = (tail_loc[0], tail_loc[1] + 1)
                elif orient == 3:
                    if parent_orient == 2:
                        intermediate_loc = (tail_loc[0] + 1, tail_loc[1])
                    else: # 4 
                        intermediate_loc = (tail_loc[0] - 1, tail_loc[1])
                elif orient == 4:
                    if parent_orient == 1:
                        intermediate_loc = (tail_loc[0], tail_loc[1] - 1)
                    else: # 3 
                        intermediate_loc = (tail_loc[0], tail_loc[1] + 1)
            
                path.append([head_loc,tail_loc,intermediate_loc])
            else: 
                path.append([head_loc,tail_loc])
           
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    if len(constraint_table) > 0:
        for constraint in constraint_table:
            # check for vertex constraints
            if len(constraint) == 2:
                if constraint[0] == next_time and constraint[1][0] == next_loc[0] and constraint[1][1] == next_loc[1]:
                    return True
                if constraint[0] < 0:
                    # A negative time constraint denotes a previous agent goal location
                    if -constraint[0] < next_time and constraint[1][0] == next_loc[0] and constraint[1][1] == next_loc[1]:
                        return True
                continue
            else:
                if next_loc is not None:  # not a vertex constraint check
                    # check for edge constraints
                    if constraint[0] == next_time \
                            and constraint[1][0] == curr_loc[0] \
                            and constraint[1][1] == curr_loc[1] \
                            and constraint[2][0] == next_loc[0] \
                            and constraint[2][1] == next_loc[1]:
                        return True
                    # check for goal constraints
#                    if ((curr_loc[0] == next_loc[0]) and (curr_loc[1] == next_loc[1])):
#                        if constraint[0] > next_time:
#                            return True
    return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node['time'], node))


def pop_node(open_list):
    _, _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def orientation(locs):
    if len(locs) == 1:  #only one location so not a multicell agent
        return 0

    # x & y were reversed here before 
    # x = locs[1][0] - locs[0][0]  # difference in x
    # y = locs[1][1] - locs[0][1]  # difference in y

    y = locs[1][0] - locs[0][0]  # difference in y
    x = locs[1][1] - locs[0][1]  # difference in x

    """
    There are 4 possible orientations for a multi-cell agent 
        Tail - Head Orientation 1 - x < 0 and y == 0
        
        Head
        Tail        Orientation 2 - x == 0 and y > 0
        
        Head - Tail Orientation 3 - x > 0 and y == 0
        
        Tail
        Head        Orientation 4 - x == 0 and y < 0
    """
    if x < 0 and y == 0:
        return 1
    if x == 0 and y > 0:
        return 2
    if x > 0 and y == 0:
        return 3
    if x == 0 and y < 0:
        return 4

def test_map(my_map, row, col, orient, dir):
    if row < 0 or row >= len(my_map):
        return True
    if col < 0 or col >= len(my_map[0]):
        return True
    if my_map[row][col]:
        return True
    if orient != 0:   # Now test if the tail is in a valid position
        row_t = row
        col_t = col
        row_t_inter = row
        col_t_inter = col 
        if orient == 1:
            col_t = col - 1

            if dir == 5:
                row_t_inter = row + 1
                col_t_inter = col_t
            elif dir == 6: 
                row_t_inter = row - 1
                col_t_inter = col_t

        if orient == 2:
            row_t = row + 1

            if dir == 5:
                row_t_inter = row_t
                col_t_inter = col + 1
            elif dir == 6: 
                row_t_inter = row_t
                col_t_inter = col - 1 

        if orient == 3:
            col_t = col + 1

            if dir == 5:
                row_t_inter = row - 1
                col_t_inter = col_t
            elif dir == 6: 
                row_t_inter = row + 1
                col_t_inter = col_t

        if orient == 4:
            row_t = row - 1

            if dir == 5:
                row_t_inter = row_t
                col_t_inter = col - 1
            elif dir == 6: 
                row_t_inter = row_t
                col_t_inter = col + 1 

        # Make sure the tail co-ordinates are on the map and the intermediate tail co-ordinates 
        if row_t < 0 or row_t >= len(my_map)\
                or col_t < 0 or col_t >= len(my_map[0])\
                 or row_t_inter < 0 or row_t_inter >= len(my_map)\
                  or col_t_inter < 0 or col_t_inter >= len(my_map[0]):
            return True
        if my_map[row_t][col_t] or my_map[row_t_inter][col_t_inter]:  # Check for collisions with the test space
            return True
        
    return False


def orient_cost(o_c, o_g):  # calculate the cost to get to the correct orientation
    if o_g == 0:
        return 0
    cost = abs(o_c - o_g)   # from 3 to 1 is 2 and vice versa 4 to 3 is 1 ...
    if cost == 3:  # but 4 to 1 is 1 90 degree rotation
        cost = 1
    return cost   # !!! calculate correct cost


def a_star(my_map, start_locs, goal_locs, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start positions
        goal_loc    - goal positions
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
        multi       - Is a this a multi-cell agent
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.
    # Build constraint table if there are constraints

    constraint_table = build_constraint_table(constraints, agent)

    open_list = []
    closed_list = dict()
    timestep = 0
    start_loc = start_locs[0]
    goal_loc = goal_locs[0]
    if len(start_locs) > 1:  # If there is more than 1 start location then this is a multi-cell agent
        multi = True
    else:
        multi = False
    
    h_value = h_values[start_loc]
    goal_orientation = orientation(goal_locs)

    root = {'loc': start_loc,'orientation': orientation(start_locs), 'g_val': 0, 'h_val': h_value, 'time': 0, 'parent': None}
    push_node(open_list, root)
    closed_list[(root['loc'], root['time'])] = root
    max_time = 5 * (len(my_map) + len(my_map[0])) # 5 time the length and width of the map should be more than enough time
    while len(open_list) > 0: #and timestep < max_time:
        curr = pop_node(open_list)
        timestep = timestep + 1
        #############################
        # If it is a multi-cell agent check for goal location and goal orientation 
        if curr['loc'] == goal_loc and curr['orientation'] == goal_orientation:
            return get_path(curr)
        ############################
        child_orient = curr['orientation']
        for dir in range(7):
            if dir < 5:
                child_loc = move(curr['loc'], dir)
            if dir == 5:
                # clockwise rotation 
                child_orient = curr['orientation'] - 1
                if child_orient < 1:
                    child_orient = 4
            if dir == 6:
                # counter-clockwise rotation 
                child_orient = curr['orientation'] + 1
                if child_orient > 4:
                    child_orient = 1
                    
            if test_map(my_map, child_loc[0], child_loc[1], child_orient, dir):
                continue

            if is_constrained(curr['loc'], child_loc, timestep, constraint_table):
                continue

            child = {'loc': child_loc,
                     'orientation': child_orient,
                     'g_val': curr['g_val'] + 1,
                     'h_val': h_values[child_loc] + orient_cost(child_orient, goal_orientation),
                     'time': timestep,
                     'parent': curr}

            if (child['loc'], child['time']) in closed_list:
                existing_node = closed_list[(child['loc'], child['time'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['time'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['time'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions
