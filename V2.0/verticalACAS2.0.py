import abc
import math
import numpy as np
import matplotlib.pyplot as plt
import heapq
import sys
from collections import deque

class Aircraft:
    def __init__(self, height, velocity, trvl_plan):
        self.height = height
        self.velocity = velocity

    def set_ac(self, name, height, velocity, trvl_plan):
        self.name = name
        self.height = height
        self.velocity = velocity
        self.trvl_plan = trvl_plan
        ac_dic = dict()
        ac_dic[name] = [height, velocity, trvl_plan]
        return ac_dic

    def height_update(self, action):
        self.volocity = action
        self.height += action * 1
        return self.height


class Graph:
    def __init__(self):
        self.x_range = height_range  # list[x_lower, x_upper, x_scale]
        self.y_range = np.array([0, end_time, time_interval])  # list[y_lower, y_upper, y_scale]

    def create_graph(self, own_name, all_ac):
        vertices = {}
        nodes_current_time = {}
        current_node = None
        intruder = all_ac.copy()
        del(intruder[own_name])
        if current_node == None:
            nodes_current_time[start] = [start, 0]
        while True:
            nodes_tmp = {}
            if current_node is not None and (current_node[0] == end_time - 2*time_interval):
                break
            for current_node in nodes_current_time:
                current_pos = nodes_current_time[current_node][0]
                vertices[current_node] = self.neighbour_vertices(own_name, current_pos, intruder)
                #neglect the difference of the impact on one child node from different parents
                nodes_tmp.update(vertices[current_node])

            nodes_current_time = nodes_tmp
        return vertices

    def neighbour_vertices(self, own_name, current_pos, intruder):
        vertices = {}
        current_time = current_pos[0]
        current_height = np.array(current_pos[1])
        neighbour_height = current_height + action_space * time_interval
        neighbour_height_idx = neighbour_height//height_range[2] * height_range[2]
        neighbour_time = current_time + time_interval
        intruder_height = []
        for key in intruder:
            intruder_height.append(intruder[key][2][neighbour_time])
        state_cost = st_cost(own_name, neighbour_height, intruder_height, neighbour_time)
        cost = state_cost + action_cost
        for i in range(len(neighbour_height_idx)):
            if neighbour_height[i] > height_range[1] or neighbour_height[i] < height_range[0]:
                continue
            vertices[(neighbour_time, neighbour_height_idx[i])] = [[neighbour_time, neighbour_height[i]], cost[i]]

        return vertices



def st_cost(own_name, neighbour_height, intruder_height, neighbour_time):
    for key in order:
        if key == own_name:
            hit_point = np.array(T_to_hit[key])
            hit_point = hit_point[np.nonzero(hit_point)]
    time_to_hit = hit_point - neighbour_time
    relative_height = np.array(neighbour_height) - np.array(intruder_height).reshape(len(intruder_height), 1)
    st_cost = np.sum(np.exp(-(relative_height / 370) ** 2) * np.exp(-(time_to_hit / 60) ** 2), 0)
    return st_cost

def shortest_path(own_name, all_ac):
    distances = {}
    previous = {}
    nodes = []
    graph = Graph()
    vertices = graph.create_graph(own_name, all_ac)

    for vertex in vertices:
        if vertex == start:
            distances[vertex] = 0
            heapq.heappush(nodes, [0, vertex])
        else:
            distances[vertex] = sys.maxsize
            heapq.heappush(nodes, [sys.maxsize, vertex])
        previous[vertex] = None

    while nodes:
        smallest = heapq.heappop(nodes)[1]
        print(smallest[0])
        sys.stdout.flush()
        if smallest[0] == end_time - time_interval:
            path = []
            while previous[smallest]:
                path.append(smallest)
                smallest = previous[smallest]
            return path
        if distances[smallest] == sys.maxsize:
            break

        for neighbour in vertices[smallest]:
            alt = distances[smallest] + vertices[smallest][neighbour][1]
            if alt < distances[neighbour]:
                distances[neighbour] = alt
                previous[neighbour] = smallest
                for n in nodes:
                    if n[1] == neighbour:
                        n[0] = alt
                        break
                    heapq.heapify(nodes)
    return distances



start = (0,0) #(time, position)
interval = 200
T_to_hit = {'ac1':[0,100],'ac2':[100,0]}
num_ac = len(T_to_hit.keys())
end_time = 200
height_range = np.array([-1500, 1500, 20])
action_space = np.array([-2500, -1500, 0, 1500, 2500])/60
action_cost = np.array([0.04, 0.02, 0, 0.02, 0.04]) * num_ac/2

init_plan = np.array([start[0]] * interval)
ac = Aircraft(0, 0, init_plan)
ac1 = ac.set_ac('ac1', 0, 0, init_plan)
ac2 = ac.set_ac('ac2', 0, 0, init_plan)
all_ac = {**ac1, **ac2}
start_height = start[0] * height_range[2] + height_range[0]

time_interval = 2
order = ['ac1', 'ac2']
if __name__ == "__main__":
    #graph = Graph()
    #vertices = Graph.create_graph('ac1', all_ac)
    shortest_path('ac1', all_ac)
    print(1)