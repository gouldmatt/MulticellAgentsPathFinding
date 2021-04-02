import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    hits = []
    length = max(len(path1), len(path2))
    #vertex collisions
    for i in range(length):
        if get_location(path1, i) == get_location(path2, i):
            # if i > len(path1) or i > len(path2):
            # if i >= len(path1):
            hits = [i - 1, get_location(path1, i)]
            # else:
            #     hits = [i - 1, get_location(path1, i)]
#            hits = [i, get_location(path1, i)]
            return hits

    for i in range(length - 1):
        if get_location(path1, i) == get_location(path2, i+1) and get_location(path1, i+1) == get_location(path2, i):
            hits = [i + 1,(get_location(path1, i)),(get_location(path1, i+1))]
            return hits
    return None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    cycles = len(paths)
    items = []
    if cycles > 1:
        for j in range(cycles - 1):
            for k in range(j + 1,cycles):
                item = []
                item = detect_collision(paths[j], paths[k])
                if item:
                    if len(item) == 2:  # vertex collisions
                       items.append({'a1': j,'a2': k, 'loc': [item[1]],'timestep': item[0]})
                    else:
                        items.append({'a1': j, 'a2': k, 'loc': [item[1],item[2]], 'timestep': item[0]})
#            hits = [{'a1': 0, 'a2' : 1, 'loc': [get_location(path1, i)], 'timestep': i}]
#    items.append({'a1': 0, 'a2': 1, 'loc': [(1,2),(1,3)], 'timestep': 3}) #test edge constraint
    return items


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    constraints = []
    constraint1 = {'agent': collision['a1'],'loc': collision['loc'], 'timestep': collision['timestep']}
    temp = collision['loc']
    if len(temp) > 1: # if it is an edge collision reverse the locations
        temp = [collision['loc'][1],collision['loc'][0]]
    constraint2 = {'agent': collision['a2'],'loc': temp, 'timestep': collision['timestep']}
    constraints.append(constraint1)
    constraints.append(constraint2)
    return constraints


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    pass

def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst

class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
#        print(node['cost'])
#        print(len(node['collisions']))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        loops = 0
        max_loops = 50  # !!! Maximum 10000 iterations
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        while len(self.open_list) > 0:
            P = self.pop_node()
            print(P['constraints'])
            if loops > max_loops:
                raise BaseException('No solutions') # maximum loops exceeded

            if len(P['collisions']) == 0: # solution found
                print(P['paths'])
                return P['paths']

            constraints = standard_splitting(P['collisions'][0])
            for c in constraints:
                Q = {'cost': 0,
                        'constraints': P['constraints'].copy(),
                        'paths': P['paths'].copy(),
                        'collisions': []}
                Q['constraints'].append(c.copy())
                agent = c['agent']
                # if c['agent'] == 1:
                #    Q['constraints'] = [{'agent': 1, 'loc': [(1, 5)], 'timestep': -4}, {'agent': 1, 'loc': [(1, 4)], 'timestep': 2},
                #                        {'agent': 1, 'loc': [(1, 3)], 'timestep': 2},
                #                        {'agent': 1, 'loc': [(1, 4)], 'timestep': 3}, {'agent': 1, 'loc': [(1, 2)], 'timestep': 1},
                #                        {'agent': 1, 'loc': [(1, 3), (1, 2)], 'timestep': 2}]
                print(Q['constraints'])
                print(Q['paths'])
                path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                              agent, Q['constraints'])
                if path is not None:
                    Q['paths'][agent] = path
                    Q['collisions'] = detect_collisions(Q['paths'])
                    Q['cost'] = P['cost'] + get_sum_of_cost(Q['paths'])
                    self.push_node(Q)
            loops = loops + 1
        raise BaseException('No solutions') #If we got here there was no solution

        self.print_results(root)
        return root['paths']


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
