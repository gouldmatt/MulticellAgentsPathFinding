import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost

def vertex_conflicts(path, test):
    conflicts = []
    end_loc = path[len(path) - 1]
    conflicts.append((-(len(path) - 1), end_loc))

    #vertex collisions
    for i in range(max(len(path),len(test))):
        if get_location(path, i) == get_location(test, i):
            if i >= len(test): # conflict is with final value of test
                conflicts.append((len(test) - 1,(get_location(test, i))))
            else:
                conflicts.append((i, (get_location(test, i))))
    return conflicts

def edge_conflicts(path, test):
    conflicts = []
    for i in range(len(test) - 1):
        if get_location(path, i) == get_location(test, i+1) and get_location(path, i+1) == get_location(test, i):
            conflicts.append((i + 1,(test[i]),(test[i+1])))
    return conflicts

def new_constraint(constraint, constraints):  #Is this a new constraint
    for straint in constraints:
        if straint['agent'] == constraint['agent'] and straint['loc'] == constraint['loc']\
            and straint['timestep'] == constraint['timestep']:
            return False

    constraints.append(constraint) # New constraint so add it
    return True # constraint doesn't exist

def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location

class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

 #       test_constraints = [{'agent':1,'loc': [(1,4)],'timestep':2} ,{'agent':1,'loc': [(1,3)],'timestep':2},
#                            {'agent':1,'loc':[(1,3),(1,2)],'timestep':2}]  # Override automated constraint generation
        test_constraints = []
        start_time = timer.time()
        result = []
        constraints = test_constraints.copy()

        for i in range(self.num_of_agents):  # Find path for each agent
            print(constraints)
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            if i + 1 < self.num_of_agents and len(test_constraints) == 0:
                agent = i + 1
                constraints = []
                conflicts = True
                attempts = 1 # keep track of attempts
                max_attempts = 2 * (len(self.my_map) + len(self.my_map[0])) # length of the perimeter of the environment
                # loop looking for a conflict free path
                while conflicts and attempts < max_attempts:
                    conflicts = False
                    attempts = attempts + 1
                    test = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], agent, constraints)
                    if result and test:
                        for path in result:
                            # Check for conflicts
                            vex_con = vertex_conflicts(path, test)
                            for c in vex_con:
                                constraint = {'agent': agent, 'loc': [c[1]], 'timestep': c[0]}
                                if new_constraint(constraint, constraints):  # Is this a new constraint
                                    conflicts = True

                            #Edge conflicts
                            edge_con = edge_conflicts(path, test)
                            for c in edge_con:
                                constraint = {'agent': agent, 'loc': [c[1], c[2]], 'timestep': c[0]}
                                if new_constraint(constraint, constraints):  # Is this a new constraint
                                    conflicts = True # New constraints added


            ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
