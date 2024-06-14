import gurobipy as gp
from gurobipy import GRB
from algorithm.base import BaseSolver
import networkx as nx


class ILPSolver(BaseSolver):
    def __init__(self, G: nx.Graph) -> None:
        super().__init__(G)

    def solve(self, k, s, t=0, time_limit=300, num_threads=2):
        self.colors = {}
        self.unique_colors = set()
        self.k = k
        self.s = s
        self.t = t
        self.x = {}
        self.y = {}
        self.z = {}

        if len(set(self.G.nodes[s]['color']) | set(self.G.nodes[t]['color'])) >= k:
            if s == t:
                self.set_solution([s])
                return

        with gp.Env(empty=True) as env:
            env.setParam('TimeLimit', time_limit)
            env.setParam('Threads', num_threads)
            env.setParam('OutputFlag', 1)
            env.start()

            with gp.Model(env=env) as model:
                self.create_transitive_closure(create_metric=True)
                self.create_variables(model)
                self.add_constraints(model)
                self.set_objective(model)

                model.optimize()

                self.set_critical_path(self.extract_solution_path(model))

    def create_variables(self, model: gp.Model):
        # Create x and y variables
        for i in self.G.nodes:
            for j in self.G.nodes:
                if i != j:
                    self.x[(i, j)] = model.addVar(vtype=GRB.BINARY)

        for i, j in self.G.edges:
            self.y[((min(i, j), max(i, j)), i)] = model.addVar(vtype=GRB.CONTINUOUS)
            self.y[((min(i, j), max(i, j)), j)] = model.addVar(vtype=GRB.CONTINUOUS)

        # Create a colors set
        for node in self.G.nodes:
            colors = self.G.nodes[node]['color']
            for color in colors:
                self.unique_colors.add(color)
                if color not in self.colors:
                    self.colors[color] = [node]
                else:
                    self.colors[color].append(node)

        # Create z
        if self.k < len(self.unique_colors):
            for color in self.unique_colors:
                self.z[color] = model.addVar(vtype=GRB.BINARY)

    def add_constraints(self, model: gp.Model):
        s, t = self.s, self.t

        # Encoding of an s-t flow:
        # 1a - outgoing flow from s
        model.addConstr(gp.quicksum(self.x[(s, v)] for v in self.G.nodes if v != s) == 1)

        # 1b - incoming flow to t
        model.addConstr(gp.quicksum(self.x[(v, t)] for v in self.G.nodes if v != t) == 1)

        # 1c - flow conservation
        for v in self.G.nodes:
            if v != s and v != t:
                model.addConstr(gp.quicksum(self.x[(u, v)] for u in self.G.nodes if u != v) == gp.quicksum(self.x[(v, u)] for u in self.G.nodes if u != v))

        # Prohibiting a circulation disjoint from s:
        # 2a - solution edge emits 2 charges
        for i, j in self.G.edges:
            if i != s and j != s:
                model.addConstr(self.y[((min(i, j), max(i, j)), i)] + self.y[((min(i, j), max(i, j)), j)] == 2 * (self.x[(i, j)] + self.x[(j, i)]))

        # 2b - charge limit per vertex
        for v in self.G.nodes:
            if v != s:
                model.addConstr(gp.quicksum(self.y[((min(u, v), max(u, v)), v)] for u in self.G.nodes if u != s and u != v) <= (2 - 1 / self.G.number_of_nodes()))

        # Counting the number of collected colors:
        # 3a - all colors are collected
        if self.k == len(self.unique_colors):
            for c in self.unique_colors:
                vertices_of_color_c = self.colors[c]
                model.addConstr(gp.quicksum(self.x[(u, v)] + self.x[(v, u)]
                                            for u in self.G.nodes
                                            for v in vertices_of_color_c
                                            if u != v) >= 1)

        # k < |C|
        if self.k < len(self.unique_colors):
            # 3b - a color is either collected or not
            for c in self.unique_colors:
                model.addConstr(self.z[c] <= 1)

            # 3c - z is 1 if any edge cover color c
            for c in self.unique_colors:
                for v in self.colors[c]:
                    for u in self.G.nodes:
                        if u != v:
                            model.addConstr(self.x[(u, v)] <= self.z[c])

            # 3d - if no edge cover color c, z is 0
            for c in self.unique_colors:
                if c not in self.colors:
                    model.addConstr(self.z[c] == 0)
                else:
                    vertices_of_color_c = self.colors[c]
                    model.addConstr(gp.quicksum(self.x[(u, v)] + self.x[(v, u)]
                                                for u in self.G.nodes
                                                for v in vertices_of_color_c
                                                if u != v) >= self.z[c])

            # 3e - must collect at least k colors
            model.addConstr(gp.quicksum(self.z[c] for c in self.unique_colors) >= self.k)

    # Objective function - minimize the weight of the solution
    def set_objective(self, model: gp.Model):
        model.setObjective(gp.quicksum(self.G.edges[i, j]['weight'] * (self.x[(i, j)] + self.x[(j, i)]) for i, j in self.G.edges),
                           GRB.MINIMIZE)

    # Extract the solution path from the model
    def extract_solution_path(self, model: gp.Model):
        if model.status == GRB.OPTIMAL:
            next_node_map = {}
            for i, j in self.x:
                if self.x[(i, j)].X > 0.9:
                    next_node_map[i] = j

            path = [self.s]
            current_node = self.s

            while len(path) == 1 or current_node != self.t:
                next_node = next_node_map[current_node]
                path.append(next_node)
                current_node = next_node

            return path
