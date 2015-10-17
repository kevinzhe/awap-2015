import networkx as nx
import random
from base_player import BasePlayer
from settings import *
import math

class Player(BasePlayer):
    """
    You will implement this class for the competition. DO NOT change the class
    name or the base class.
    """

    # You can set up static state here
    has_built_station = False
    distance_heuristic = 'MANHATTAN'

    def __init__(self, state):
        """
        Initializes your Player. You can set up persistent state, do analysis
        on the input graph, engage in whatever pre-computation you need. This
        function must take less than Settings.INIT_TIMEOUT seconds.
        --- Parameters ---
        state : State
            The initial state of the game. See state.py for more information.
        """

        G = state.get_graph()
        
        #Getting the vertex with the maximum degree
        nodes = G.nodes()
        maxDegree = 0
        self.maxVertex = [] #List of vertices with the maximum degree
        for vtx in nodes:
            degree = G.degree(vtx)
            if degree > maxDegree:
                maxDegree = degree
                self.maxVertex = [vtx]
            elif degree == maxDegree:
                self.maxVertex.append(vtx)

        # pick closest to center
        # TODO: maybe change to heuristic
        self.sideLen = int(math.ceil(math.sqrt(GRAPH_SIZE)))
        center = GRAPH_SIZE / 2
        minDist = None
        bestNode = None
        for v in self.maxVertex:
            dist = self.get_distance(center, v)
            if minDist is None or dist < minDist:
                bestNode = v
                minDist = dist

        self.stations = [bestNode]
        self.to_build = [bestNode]

        self.build_cost = INIT_BUILD_COST
        
        self.degreeDict = dict()
        for vtx in nodes:
            degree = G.degree(vtx)
            if degree not in self.degreeDict:
                self.degreeDict[degree] = set()
            self.degreeDict[degree].add(vtx)
            
        return

    # get the distance between two nodes
    def get_distance(self, node1, node2):
        x1, y1 = node1 / self.sideLen, node1 % self.sideLen
        x2, y2 = node2 / self.sideLen, node2 % self.sideLen
        if self.distance_heuristic == 'MANHATTAN':
            return abs(x1 - x2) + abs(y1 - y2)
        else:
            return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    # Checks if we can use a given path
    def path_is_valid(self, state, path):
        graph = state.get_graph()
        for i in range(0, len(path) - 1):
            if graph.edge[path[i]][path[i + 1]]['in_use']:
                return False
        return True

    def step(self, state):
        """
        Determine actions based on the current state of the city. Called every
        time step. This function must take less than Settings.STEP_TIMEOUT
        seconds.
        --- Parameters ---
        state : State
            The state of the game. See state.py for more information.
        --- Returns ---
        commands : dict list
            Each command should be generated via self.send_command or
            self.build_command. The commands are evaluated in order.
        """

        commands = []
        new_to_build = []
        for s in self.to_build:
            if self.money >= self.build_cost
                commands.append(self.build_command(s))
                self.money -= self.build_cost
                self.build_cost *= BUILD_FACTOR
            else:
                new_to_build.append(s)

        self.to_build = new_to_build

        return commands
