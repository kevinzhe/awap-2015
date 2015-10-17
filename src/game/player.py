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

    # order money
    money_threshold = 15

    # order counting
    order_counts = None

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

        self.distances = nx.all_pairs_shortest_path_length(G)

        self.money = STARTING_MONEY

        self.last_build = 0
        self.build_money = STARTING_MONEY

        self.edges = G.edges()
        self.pathLenth = [self.get_distance(edge[0], edge[1]) for edge in self.edges]
        self.maxPathLength = max(self.pathLenth)
        return



    # get the distance between two nodes
    def get_distance(self, node1, node2):
        return self.distances.get(node1).get(node2)

    # check distance between destination with most degree and curStation
    def destinationCounts(self, state):
        pending = state.get_pending_orders()
        active = state.get_active_orders()
        orders = map(lambda order: order.get_node(), pending)
        orders.extend(map(lambda t: t[0].get_node(), active))
        nodeCounts = dict()
        for node in orders:
            if node not in nodeCounts: 
                nodeCounts[node] = 0
            nodeCounts[node] += 1
        return nodeCounts


    #return the next node where we can build a station
    def get_next_station_node(self, state):
        destinations = self.destinationCounts(state)
        maxCount = 0
        curCount = 0
        station = None
        for node in destinations:
            curCount = destinations[node]
            if node not in self.stations:
                if curCount >= maxCount:
                    maxCount = curCount
                    station = node
        return station

    def hasCloseNeighbor(self, state, station):
        G = state.get_graph()
        for node in self.stations:
            distance = self.get_distance(station, node)
            if distance < self.maxPathLength/4:
                return True
        return False

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
        
        if (len(self.stations) < HUBS and
            self.money >= self.build_cost and
            state.get_time() != self.last_build and
            self.money != self.build_money):
            slope = 1.0 * ((self.money - self.build_money) /
                           (state.get_time() - self.last_build))
            est_slope = slope * self.station_factor
            est_time = self.find_time_overlap(slope, est_slope, state.get_time())
            # print "    ", est_time
        else:
            est_time = 10000

        if (est_time < 1000):
            s = self.get_next_station_node(state)
            if s != None:
                self.to_build.append(s)

        commands = []

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

        station_order_pairs = map(lambda a: (a[0], a[1], self.score_order(a[0], a[1], state)),
            list(itertools.product(
            state.get_pending_orders(), self.stations)))

        station_order_pairs = filter(lambda a: a[2] > self.money_threshold, station_order_pairs)

        station_order_pairs.sort(lambda a,b: self.cmp(a, b))

        paths = set()
        orders_done = set()

        for (order, station, _) in station_order_pairs:
            if (order not in orders_done):
                path = self.find_path(G, station, order.get_node(), paths)
                if path != None:
                    commands.append(self.send_command(order, path))
                    self.add_path(paths, path)
                    orders_done.add(order)

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

    def cmp(self, a, b):
        # print "    ", a, b
        _, _, a = a
        _, _, b = b
        return -cmp(a, b)

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
    
    def update_orders(state):
        '''Return a list of ({PENDING},{FUFILLED}) tuples'''
        if self.order_counts is None:
            self.order_counts = [(set(), set()) for _ in range(GRAPH_SIZE)]
        for o in state.get_pending_orders():
            (p,f) = self.order_counts[o.node]
            p.add(o.id)
        for o in state.get_active_orders():
            (p,f) = self.order_counts[o.node]
            f.add(o.id)

    def get_order_count(node_num):
        p,f = self.order_counts[node_num]
        return len(p) + len(f)


