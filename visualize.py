#!/usr/bin/env python3
from matplotlib.patches import Circle, Rectangle, FancyArrowPatch 
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation

Colors = ['green', 'blue', 'orange']


class Animation:
    def __init__(self, my_map, starts, goals, paths):
        self.my_map = np.flip(np.transpose(my_map), 1)

        num_cols = len(self.my_map[0])

        # construct starting locations list 
        self.starts = []
        for start in starts:
            multi_cells = []
            for cell in start: 
                multi_cells.append((cell[1], num_cols - 1 - cell[0]))
            self.starts.append(multi_cells)
        
        # construct goal locations list 
        self.goals = []
        for goal in goals:
            multi_cells = []
            for cell in goal: 
                multi_cells.append((cell[1], num_cols - 1 - cell[0]))
            self.goals.append(multi_cells)

        # construct list of agent paths 
        self.paths = []
        self.text_locs = []
        self.rotations = []
        if paths:
            for path in paths:
                self.paths.append([])
                self.rotations.append([])
                self.text_locs.append([])

                if type(path[0][0]) is int: 
                    # single cell 
                    last_rotation = 0
                    degree = 0
                else:
                    last_rotation = self.find_rotation(path[0])
                    degree = self.find_degree(last_rotation)

                for loc in path:
                    if type(loc[0]) is int: 
                        # single cell 
                        self.paths[-1].append((loc[1], len(self.my_map[0]) - 1 - loc[0]))
                        continue

                    rotation = self.find_rotation(loc)

                    # determine current rotation degree of agent     
                    if rotation == 4 and last_rotation == 1:
                        # print("clockwise")
                        degree = degree - 90
                        self.rotations[-1].append(degree)

                    elif rotation == 1 and last_rotation == 4:
                        # print("counter")
                        degree = degree + 90
                        self.rotations[-1].append(degree)

                    elif last_rotation > rotation:
                        # print("clockwise")
                        degree = degree - 90
                        self.rotations[-1].append(degree)
                    elif last_rotation < rotation:
                        # print("counter")
                        degree = degree + 90
                        self.rotations[-1].append(degree)
                    else:
                        self.rotations[-1].append(degree)

                    # find xy to plot agent rectangle at for given location and rotation 
                    pos = self.find_xy(loc[0],rotation)
            
                    x = pos[1] + 0.125
                    y = num_cols - 1 - pos[0] + 0.25
   
                    self.paths[-1].append((x,y))
                    last_rotation = rotation

                    # find xy to plot agent text at for given location and rotation 
                    pos = self.find_xy_text(loc[0],rotation)
            
                    x = pos[1] + 0.10
                    y = num_cols - 1 - pos[0] - 0.125
   
                    self.text_locs[-1].append((x,y))

                    # save rotation for next iteration 
                    last_rotation = rotation


        aspect = len(self.my_map) / len(self.my_map[0])

        self.fig = plt.figure(frameon=False, figsize=(4 * aspect, 4))
        self.ax = self.fig.add_subplot(111, aspect='equal')
        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1, wspace=None, hspace=None)

        self.patches = []
        self.artists = []
        self.agents = dict()
        self.agent_names = dict()
   
        # create boundary patch
        x_min = -0.5
        y_min = -0.5
        x_max = len(self.my_map) - 0.5
        y_max = len(self.my_map[0]) - 0.5
        plt.xlim(x_min, x_max)
        plt.ylim(y_min, y_max)

        # draw map rectangles 
        self.patches.append(Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, facecolor='none', edgecolor='gray'))
        for i in range(len(self.my_map)):
            for j in range(len(self.my_map[0])):
                if self.my_map[i][j]:
                    self.patches.append(Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='gray', edgecolor='gray'))

        self.T = 0

        # draw goals first
        for i, goal in enumerate(self.goals):
            for cell in goal: 
                self.patches.append(Rectangle((cell[0] - 0.25, cell[1] - 0.25), 0.5, 0.5, facecolor=Colors[i % len(Colors)],
                                            edgecolor='black', alpha=0.5))
            
        # going through and adding agents 
        for i in range(len(self.paths)): 
            # check for single cell agent 
            if type(self.paths[i][0][0]) is int:  
                name = str(i)
                self.agents[i] = Circle((starts[i][0][0], starts[i][0][1]), 0.3, facecolor=Colors[i % len(Colors)],
                                        edgecolor='black')
                self.agents[i].original_face_color = Colors[i % len(Colors)]
                self.patches.append(self.agents[i])
                self.T = max(self.T, len(paths[i]) - 1)
                self.agent_names[i] = self.ax.text(starts[i][0][0], starts[i][0][1] + 0.25, name)
                self.agent_names[i].set_horizontalalignment('center')
                self.agent_names[i].set_verticalalignment('center')
                self.artists.append(self.agent_names[i])
                continue
                
            # multi cell agent with rectangle rather than circle 
            name = str(str(i))
            rotation = self.find_rotation(starts[i])
            xy = self.find_xy(starts[i][0],rotation)
            x = xy[1] 
            y = num_cols - 1 - xy[0]
            degree = self.find_degree(rotation)

            self.agents[i] = Rectangle((x,y), 1.25, 0.5, angle=degree, facecolor=Colors[i % len(Colors)],
                                            edgecolor='black', alpha=1, label = "round")

            self.agents[i].original_face_color = Colors[i % len(Colors)]
            
            self.patches.append(self.agents[i])
            self.T = max(self.T, len(paths[i]) - 1)
            self.agent_names[i] = self.ax.text(self.text_locs[i][0][0], self.text_locs[i][0][1] + 0.25, name)
            self.agent_names[i].set_horizontalalignment('center')
            self.agent_names[i].set_verticalalignment('center')
            self.artists.append(self.agent_names[i])
        

        self.animation = animation.FuncAnimation(self.fig, self.animate_func,
                                                 init_func=self.init_func,
                                                 frames=int(self.T + 1) * 10,
                                                 interval=100,
                                                 blit=True)

    # find xy to plot agent rectangle at for given head location and rotation 
    def find_xy(self,head,rotation):
        if(rotation == 1):
            xy = (head[0] , head[1] )
        elif(rotation ==2):
            xy = (head[0] + 0.25 , head[1] - 0.375)
        elif(rotation == 3):
            xy = (head[0] + 0.5, head[1] - 0.25 )
        else: 
            xy = (head[0] + 0.375, head[1] + 0.125)
        return xy
      
    # find xy to plot agent text at for given head location and rotation 
    def find_xy_text(self,head,rotation):
        if(rotation == 1):
            xy = (head[0], head[1] - 0.5 )
        elif(rotation ==2):
            xy = (head[0] + 0.25 , head[1] - 0.125 )
        elif(rotation == 3):
            xy = (head[0], head[1] + 0.5 )
        else: 
            xy = (head[0] + 0.65 , head[1] - 0.125 )
        return xy

    # find the degree that the rectangle swatch is 
    # rotated at 
    # https://matplotlib.org/stable/api/_as_gen/matplotlib.patches.Rectangle.html
    def find_degree(self,rotation):
        if(rotation == 1):
            degree = 180
        elif(rotation ==2):
            degree = 270
        elif(rotation == 3):
            degree = 0
        else: 
            degree = 90
            
        return degree 

    # same process as orientation() in single_agent_planner.py 
    def find_rotation(self, locs):
        y = locs[1][0] - locs[0][0]  # difference in y
        x = locs[1][1] - locs[0][1]  # difference in x

        if x < 0 and y == 0:
            return 1
        if x == 0 and y > 0:
            return 2
        if x > 0 and y == 0:
            return 3
        if x == 0 and y < 0:
            return 4


    def save(self, file_name, speed):
        self.animation.save(
            file_name,
            fps=10 * speed,
            dpi=200,
            savefig_kwargs={"pad_inches": 0, "bbox_inches": "tight"})

    @staticmethod
    def show():
        plt.show()

    def init_func(self):
        for p in self.patches:
            self.ax.add_patch(p)
        for a in self.artists:
            self.ax.add_artist(a)
        return self.patches + self.artists

    def animate_func(self, t):
        for k in range(len(self.paths)):
            if type(self.paths[k][0][0]) is int: 
                pos = self.get_state(t / 10, self.paths[k])
                self.agents[k].center = (pos[0], pos[1])
                self.agent_names[k].set_position((pos[0], pos[1] + 0.5))
                continue

            # update the agent location and rotation 
            pos = self.get_state(t / 10, self.paths[k])
            rotate = self.get_orientation(t / 10, self.rotations[k])
            
            self.agents[k].xy = (pos[0], pos[1])
            self.agents[k].angle = rotate
            self.agents[k].set_label("test")

            # update the agent text label location and visbility 
            pos_text = self.get_text_xy(t/10, self.text_locs[k])
            if(rotate % 90 == 0):
                self.agent_names[k].set_visible(True)
            else:
                self.agent_names[k].set_visible(False)      

            self.agent_names[k].set_position((pos_text[0], pos_text[1] + 0.5))
           

        # reset all colors
        for _, agent in self.agents.items():
            agent.set_facecolor(agent.original_face_color)

        # check drive-drive collisions
        agents_array = [agent for _, agent in self.agents.items()]
        for i in range(0, len(agents_array)):
            for j in range(i + 1, len(agents_array)):
                d1 = agents_array[i]
                d2 = agents_array[j]

                # need to implement collisions detection
                # for multi cell agent to highlight in animation 
                if(isinstance(d1, Rectangle)):
                    continue

                if(isinstance(d2, Rectangle)):
                    continue

                pos1 = np.array(d1.center)
                pos2 = np.array(d2.center)
                if np.linalg.norm(pos1 - pos2) < 0.7:
                    d1.set_facecolor('red')
                    d2.set_facecolor('red')
                    print("COLLISION! (agent-agent) ({}, {}) at time {}".format(i, j, t/10))

        return self.patches + self.artists

    @staticmethod
    def get_orientation(t, rotations):
        if(int(t) > len(rotations) -1 ):
            return(rotations[-1])
        elif int(t) <= 0: 
            return(rotations[0])
        else:
            rot_last = np.array(rotations[int(t) - 1])
            rot_next = np.array(rotations[int(t) ])
            return((rot_next - rot_last) * (t - int(t)) + rot_last)


    @staticmethod
    def get_text_xy(t, text_locs):
        if int(t) <= 0:
            return np.array(text_locs[0])
        elif int(t) >= len(text_locs):
            return np.array(text_locs[-1])
        else:
            pos_last = np.array(text_locs[int(t) - 1])
            pos_next = np.array(text_locs[int(t)])
            pos = (pos_next - pos_last) * (t - int(t)) + pos_last
            return pos


    @staticmethod
    def get_state(t, path):
        if int(t) <= 0:
            return np.array(path[0])
        elif int(t) >= len(path):
            return np.array(path[-1])
        else:
            pos_last = np.array(path[int(t) - 1])
            pos_next = np.array(path[int(t)])
            pos = (pos_next - pos_last) * (t - int(t)) + pos_last
            return pos


   