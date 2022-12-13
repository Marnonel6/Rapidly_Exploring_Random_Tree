from calendar import c
from lib2to3.pgen2.token import OP
from turtle import circle
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.collections import CircleCollection
import random


"""                             """
"""         RRT Class           """
"""                             """

class RRT:
    # Variables 
    def __init__(self):
        self.vertices_list = [] # (G) Has form [Node name, [node coordinate], [node parent cooordinate]]
        self.edge = [] # Holds the edges/line coordinates of the nodes/vertices
        self.K = 1500 # Number of itterations/vertices/nodes
        self.q_goal = [] # Goal position
        self.q_init = [] # Starting position
        self.q_near = [] # This is the nearest node to the random point
        self.q_rand = [] # Contains the new random node from "def rand_point(self)"
        self.q_new = [] # Used to store the new node on a unit vecto
        self.delta = 1 # Move distance for each step from parent to new child
        # Explore domain/matrix
        self.D = np.zeros((100,100)) # 2D domain to explore


    """                             """
    """         Functions           """
    """                             """
    # Creates a random starting point
    def Random_starting_point(self):
        rand_point_x = np.random.randint(0,self.D.shape[0])
        rand_point_y = np.random.randint(0,self.D.shape[1])
        return [rand_point_x, rand_point_y]

    # Creates a random goal
    def Random_goal_point(self):
        rand_point_x = np.random.randint(0,self.D.shape[0])
        rand_point_y = np.random.randint(0,self.D.shape[1])
        return [rand_point_x, rand_point_y]


    # Creates a random point
    def Random_configuration(self):
        rand_point_x = np.random.randint(0,self.D.shape[0])
        rand_point_y = np.random.randint(0,self.D.shape[1])
        return [rand_point_x, rand_point_y]

    
    # Finds the closest point on a ine relative to a coordinate
    def closest_point(self, p1, p2, p3):
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        dx, dy = x2-x1, y2-y1
        det = dx*dx + dy*dy
        a = (dy*(y3-y1)+dx*(x3-x1))/det
        return [x1+a*dx, y1+a*dy], a





    # Determins the nearest node and adds it to the vertices list in this config:
    # [node_name,[new node coordinates],[parent node coordinates]]
    def Nearest_node(self, q_rand, vertices_list):

        distance_node = [] # Distance between new node and all the existing nodes

        # For loop for calculating the distance between the random point and all the nodes that exists
        for i in range(0,len(vertices_list)): 
            # Pythagorean theorem
            x = np.square(q_rand[0]-vertices_list[i][1][0])
            y = np.square(q_rand[1]-vertices_list[i][1][1])
            pythag = np.sqrt(x+y)
            distance_node.append(pythag)

        # Returns the minimum distance and the index of the min distance in the list distance_node
        min_dist = min(distance_node)
        min_index = distance_node.index(min_dist)
        # GET PARENT
        parent = vertices_list[min_index][1]

        # Return parent/q_near
        return parent

    def New_configuration(self, q_near, q_rand, delta):
        # Calculate the unit vector
        distance = [q_rand[0] - q_near[0], q_rand[1] - q_near[1]]
        norm = np.sqrt([(np.square(q_rand[0] - q_near[0])) + (np.square(q_rand[1] - q_near[1]))])
        unit_vector = [distance[0]/norm[0], distance[1]/norm[0]]
        new_node = [q_near[0] + unit_vector[0], q_near[1] + unit_vector[1]]

        return new_node

"""                                           """
"""         Random Circle Generator Class     """
"""                                           """
class Random_Circle_Generator:
    # Variables 
    def __init__(self):
        self.circle_list = [] # Has form [[center], [radius]] - Nested list
        self.C = 50 # Number of Circles
        self.c_rand = [] # Contains the new random circle parameters [[center],[radius]] from "def rand_circle(self)"
        self.max_radius = 10 # Maximum radius that can be created
    
        """                             """
        """         Functions           """
        """                             """
    # Creates a random point
    def Random_circle_configuration(self, D):
        rand_centre_x = np.random.randint(0,D.shape[0])
        rand_centre_y = np.random.randint(0,D.shape[1])
        rand_radius = np.random.randint(3,self.max_radius+1) # '+1' because high is exlusive and low is inclusive
        return [[rand_centre_x, rand_centre_y], rand_radius]

"""                                                                                               """
"""                                    Main Body Code Starts                                      """
"""                                                                                               """
RRT1 = RRT() # Create an object of class RRT
RCG1 = Random_Circle_Generator() # Create onject for random circle generator

"""                                                """
"""         Random circle generator code           """
"""                                                """
n = 0 # Counter for creating X amount of circles (Needs to only be updated when a valid circle is created)

while n < RCG1.C:
    n+=1 # Move to update only when valid cicrcle is generated
    RCG1.circle_list.append(RCG1.Random_circle_configuration(RRT1.D))

"""                                                                     """
"""   Collision detection/prevention algoritm for START & GOAL point    """
"""                                                                     """
# Generate random starting point and check if point is in a circle if it is generate a new starting point, if it isn't break while
s = 0
while s<1:

    s = 1 # Set check to false to ensure while loop is broken if the 'if' condition is not met and thus the point is not in any circle
    RRT1.q_init = RRT1.Random_starting_point()

    for i in range(0, len(RCG1.circle_list)):
        # If distance between start point and a circles radius <= that circle raius then start point is in circle, thus get a new start point.
        if (np.sqrt(np.square(RRT1.q_init[0]-RCG1.circle_list[i][0][0]) + (np.square(RRT1.q_init[1]-RCG1.circle_list[i][0][1])))) <= RCG1.circle_list[i][1]:
            s = 0

p = 0
while p<1:

    p = 1 # Set check to false to ensure while loop is broken if the 'if' condition is not met and thus the point is not in any circle
    RRT1.q_goal = RRT1.Random_goal_point()

    for i in range(0, len(RCG1.circle_list)):
        # If distance between goal point and a circles radius <= that circle raius then goal point is in circle, thus get a new goal point.
        if (np.sqrt(np.square(RRT1.q_goal[0]-RCG1.circle_list[i][0][0]) + (np.square(RRT1.q_goal[1]-RCG1.circle_list[i][0][1])))) <= RCG1.circle_list[i][1]:
            p = 0

# Add starting position/node to vertice list
RRT1.vertices_list.append([len(RRT1.vertices_list), RRT1.q_init, RRT1.q_init])

"""                                                """
"""         While iteration loop for RRT           """
"""                                                """

t = 0 # counting var for While
Total_iterations = 0
# MAIN while loop
while t < RRT1.K:
    t += 1 # Increment the counter

    Total_iterations += 1

    """                                         """
    """ Collision detection/prevention algoritm """
    """                                         """
    z = 0
    
    while z < 1:
        z = 1
        n = 1 # for goal check
        # Generate a random point
        RRT1.q_rand = RRT1.Random_configuration()

        # Get nearest vertex
        RRT1.q_near = RRT1.Nearest_node(RRT1.q_rand, RRT1.vertices_list) # Send the new random node and the vertices list
        RRT1.q_new = RRT1.New_configuration(RRT1.q_near, RRT1.q_rand, RRT1.delta)


        for i in range(0, len(RCG1.circle_list)):

            """ Checks only point intersect not line """ 
            # If distance between start point and a circles radius <= that circle raius then start point is in circle, thus get a new start point.
            if (np.sqrt(np.square(RRT1.q_new[0]-RCG1.circle_list[i][0][0]) + np.square(RRT1.q_new[1]-RCG1.circle_list[i][0][1]))) <= RCG1.circle_list[i][1]:
                z = 0

            # For goal check
            """ Check closest point intersect using a line """ 
            [p_close, a] = RRT1.closest_point(RRT1.q_new, RRT1.q_goal, RCG1.circle_list[i][0])
            # If distance between start point and a circles radius <= that circle radius then start point is in circle, thus get a new start point.
            if a <= 1 and a >= 0: # Limits the shortest distance line check to between the two coordinates thus not checking the line extending past the two coordinates
                if (np.sqrt(np.square(p_close[0]-RCG1.circle_list[i][0][0]) + np.square(p_close[1]-RCG1.circle_list[i][0][1]))) <= RCG1.circle_list[i][1]:
                     n = 0

        # for goal check
        if n == 1:
            t = RRT1.K
            print('Goal Reached')
            # 1ste arg: Name of new node, 2nd argument: New child/node cooordinate, 3rd argument: Parent coordinates
            RRT1.vertices_list.append([len(RRT1.vertices_list), RRT1.q_new, RRT1.q_near]) 
            RRT1.vertices_list.append([len(RRT1.vertices_list), RRT1.q_goal, RRT1.q_new]) 

    if n == 0: # for goal check
        # 1ste arg: Name of new node, 2nd argument: New child/node cooordinate, 3rd argument: Parent coordinates
        RRT1.vertices_list.append([len(RRT1.vertices_list), RRT1.q_new, RRT1.q_near]) 

print('\n Total_iterations =', Total_iterations)  

Optimal_path = []
# Find the last node or goal node in RRT1.vertices_list
retrieved_elements = list(filter(lambda x: RRT1.q_goal in x, RRT1.vertices_list[:]))
print(retrieved_elements,'\n')
print('goal = ', RRT1.q_goal)
Optimal_path.append([retrieved_elements[0][1], retrieved_elements[0][2]])
print('Optimal_path = ', Optimal_path)

s = 0 # Counter
while Optimal_path[s][1] != RRT1.q_init:
#for w in range(3):
    retrieved_elements = list(filter(lambda x: Optimal_path[s][1] in x, RRT1.vertices_list[:][:])) # Find current node parent
    Optimal_path.append([retrieved_elements[0][1], retrieved_elements[0][2]])
    s += 1

"""                                """
"""         Plotting RRT           """
"""                                """
                    ### PLOT LINE COLLECTION ###
# create random data
data = RRT1.vertices_list
line_data = [] # Used to store each line segments two point - child and parent

for i in range(0,len(data)):
    line_data.append([data[i][1],data[i][2]]) #First one is the child and the second one is the parent

lc = LineCollection(line_data) # Make into a collection

# plot the data
fig = plt.figure()
ax = fig.add_subplot()
ax.add_collection(lc)

                    ### Plot path in RED ###
O_path = LineCollection(Optimal_path, color = 'Red') # Make into a collection
ax.add_collection(O_path )

                    ### Plot start point ###
start_point_radius = [1] # Choose start point size
Draw_start_point = plt.Circle((RRT1.q_init[0],RRT1.q_init[1]), start_point_radius[0], color = 'Green')
ax.add_artist( Draw_start_point )

                    ### Plot goal point ###
goal_point_radius = [1] # Choose goal point size
Draw_goal_point = plt.Circle((RRT1.q_goal[0],RRT1.q_goal[1]), goal_point_radius[0], color = 'Blue')
ax.add_artist( Draw_goal_point )

                    ### PLOT CIRCLE COLLECTION ###

for i in range(0,len(RCG1.circle_list)):
    """Single random circle plot"""
    Drawing_colored_circle = plt.Circle(( RCG1.circle_list[i][0][0] , RCG1.circle_list[i][0][1] ), RCG1.circle_list[i][1], color = 'black' )
    ax.add_artist( Drawing_colored_circle )

# set the limits
ax.set_xlim([0, 100])
ax.set_ylim([0, 100])

ax.set_title('Rapid Exploring Random Tree')

# display the plot
plt.show()
