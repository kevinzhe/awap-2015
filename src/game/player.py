import networkx as nx
import random
from base_player import BasePlayer
from settings import *
import math
import heapq
import itertools

class Player(BasePlayer):
    """
    You will implement this class for the competition. DO NOT change the class
    name or the base class.
    """

    # You can set up static state here
    has_built_station = False
    distance_heuristic = 'MANHATTAN'

    # Heuristic Weightings
    distance_weight = 10
    degree_weight = 10
    station_weight = 100

    # station building
    station_factor = 1.2

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

        self.stations = []
        self.to_build = [self.maxVertex[len(self.maxVertex) / 2]]

        self.build_cost = INIT_BUILD_COST
        
        self.degreeDict = dict()
        for vtx in nodes:
            degree = G.degree(vtx)
            if degree not in self.degreeDict:
                self.degreeDict[degree] = set()
            self.degreeDict[degree].add(vtx)

        self.distances = nx.all_pairs_shortest_path_length(G,
            ((SCORE_MEAN + SCORE_VAR) / DECAY_FACTOR)
        )

        self.money = STARTING_MONEY

        self.last_build = 0
        self.build_money = STARTING_MONEY

        return

    # get the distance between two nodes
    def get_distance(self, node1, node2):
        return self.distances.get(node1).get(node2)

    # Checks if we can use a given path
    def path_is_valid(self, state, path):
        graph = state.get_graph()
        for i in range(0, len(path) - 1):
            if graph.edge[path[i]][path[i + 1]]['in_use']:
                return False
        return True

    def step(self, state):
        # print self.money, self.build_money, self.build_cost
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

        G = state.get_graph()
        self.money = state.get_money()

        if (self.money >= self.build_cost and
            state.get_time() != self.last_build and
            self.money != self.build_money):
            slope = 1.0 * ((self.money - self.build_money) /
                           (state.get_time() - self.last_build))
            est_slope = slope * self.station_factor
            est_time = self.find_time_overlap(slope, est_slope, state.get_time())
            # print "    ", est_time
        else:
            est_time = 10000

        commands = []

        if (est_time < 1000):
            self.to_build.append(self.find_new_station())

        new_to_build = []
        for s in self.to_build:
            if self.money >= self.build_cost:
                commands.append(self.build_command(s))
                self.money -= self.build_cost
                self.build_cost *= BUILD_FACTOR
                self.stations.append(s)
                self.last_build = state.get_time()
                self.build_money = self.money
            else:
                new_to_build.append(s)

        self.to_build = new_to_build

        station_order_pairs = list(itertools.product(
            state.get_pending_orders(), self.stations))

        station_order_pairs.sort(lambda a,b: self.cmp(a, b, state))

        paths = set()

        for (order, station) in station_order_pairs:
            path = self.find_path(G, station, order.get_node(), paths)
            if path != None:
                commands.append(self.send_command(order, path))
                self.add_path(paths, path)

        return commands

    def find_new_station(self):
        return random.randint(0, GRAPH_SIZE - 1)

    def find_time_overlap(self, m1, m2, t2):
        s = self.money
        c = self.build_cost
        t1 = self.last_build
        p = self.build_money
        return 1.0 * (s + m1 * t1 - m2 * t2 - c - p) / (m1 - m2)

    def add_path(self, paths, path):
        for i in range(len(path) - 1):
            paths.add((path[i], path[i+1]))
            paths.add((path[i+1], path[i]))

    def cmp(self, a, b, state):
        oa, sa = a
        ob, sb = b
        return -cmp(self.score_order(oa, sa, state), self.score_order(ob, sb, state))

    def score_order(self, order, station, state):
        d = self.get_distance(order.get_node(), station)
        if d == None:
            return 0
        else:
            return (order.get_money() - DECAY_FACTOR *
                (state.time - order.get_time_created() + d))

    def score_node(self, G, node, dest):
        # low score is good
        score = 0
        # prefer short paths
        d = self.get_distance(node, dest)
        if d != None:
            score += self.distance_weight * d
        else:
            score += 10**15
        # prefer low degree nodes
        score += self.degree_weight * len(G.neighbors(node))
        # don't go through other stations
        if G.node[node]['is_station']: score += self.station_weight

        return score

    def find_path(self, G, src, dest, paths):
        # a star
        queue = []
        heapq.heappush(queue, [0, src, [src]])
        seen = set()
        while len(queue) > 0:
            _, node, path = heapq.heappop(queue)
            if node in seen: continue
            seen.add(node)
            if node == dest:
                return path
            neighbors = G.neighbors(node)
            for neighbor in neighbors:
                if not (G.edge[node][neighbor]['in_use'] or 
                        (node, neighbor) in paths or
                        (neighbor, node) in paths):
                    heapq.heappush(queue, [
                        self.score_node(G, neighbor, dest),
                        neighbor,
                        path + [neighbor],
                    ])
        return None
