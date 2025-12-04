#!/usr/bin/env python3
"""
Route Planner with Dijkstra, A*, and Bellman-Ford algorithms
"""

import sys
import csv
import heapq
import math
from typing import Dict, List, Tuple, Optional

EARTH_RADIUS = 6371.0  # km


class Node:
    """Represents a node in the graph"""
    def __init__(self, node_id: int, lat: float, lon: float, early: int, late: int):
        self.id = node_id
        self.lat = lat
        self.lon = lon
        self.early = early
        self.late = late


class Edge:
    """Represents an edge in the graph"""
    def __init__(self, to: int, weight: float):
        self.to = to
        self.weight = weight


class Graph:
    """Graph data structure with adjacency list"""
    def __init__(self):
        self.nodes: Dict[int, Node] = {}
        self.adj_list: Dict[int, List[Edge]] = {}
    
    def add_node(self, node_id: int, lat: float, lon: float, early: int, late: int):
        """Add a node to the graph"""
        self.nodes[node_id] = Node(node_id, lat, lon, early, late)
        if node_id not in self.adj_list:
            self.adj_list[node_id] = []
    
    def add_edge(self, from_id: int, to_id: int, weight: float):
        """Add an edge to the graph"""
        if from_id not in self.adj_list:
            self.adj_list[from_id] = []
        self.adj_list[from_id].append(Edge(to_id, weight))


def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculate haversine distance between two points"""
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return EARTH_RADIUS * c


def dijkstra(graph: Graph, start: int, end: int) -> Tuple[Dict[int, float], Dict[int, Optional[int]], int]:
    """
    Dijkstra's algorithm for shortest path
    Returns: (distances, previous nodes, nodes explored)
    """
    dist = {node_id: float('inf') for node_id in graph.nodes}
    prev = {node_id: None for node_id in graph.nodes}
    dist[start] = 0
    
    pq = [(0, start)]
    nodes_explored = 0
    visited = set()
    
    while pq:
        current_dist, u = heapq.heappop(pq)
        
        if u in visited:
            continue
        
        visited.add(u)
        nodes_explored += 1
        
        if u == end:
            break
        
        if current_dist > dist[u]:
            continue
        
        for edge in graph.adj_list.get(u, []):
            v = edge.to
            alt = dist[u] + edge.weight
            
            if alt < dist[v]:
                dist[v] = alt
                prev[v] = u
                heapq.heappush(pq, (alt, v))
    
    return dist, prev, nodes_explored


def astar(graph: Graph, start: int, end: int) -> Tuple[Dict[int, float], Dict[int, Optional[int]], int]:
    """
    A* algorithm for shortest path
    Returns: (distances, previous nodes, nodes explored)
    """
    dist = {node_id: float('inf') for node_id in graph.nodes}
    prev = {node_id: None for node_id in graph.nodes}
    dist[start] = 0
    
    end_node = graph.nodes[end]
    
    def heuristic(node_id: int) -> float:
        node = graph.nodes[node_id]
        return haversine(node.lat, node.lon, end_node.lat, end_node.lon)
    
    pq = [(heuristic(start), start)]
    nodes_explored = 0
    visited = set()
    
    while pq:
        _, u = heapq.heappop(pq)
        
        if u in visited:
            continue
        
        visited.add(u)
        nodes_explored += 1
        
        if u == end:
            break
        
        for edge in graph.adj_list.get(u, []):
            v = edge.to
            alt = dist[u] + edge.weight
            
            if alt < dist[v]:
                dist[v] = alt
                prev[v] = u
                f_score = alt + heuristic(v)
                heapq.heappush(pq, (f_score, v))
    
    return dist, prev, nodes_explored


def astar_time(graph: Graph, start: int, end: int) -> Tuple[Dict[int, float], Dict[int, Optional[int]], int]:
    """
    A* algorithm for shortest path that respects time intervals
    Returns: (distances, previous nodes, nodes explored)
    """
    dist = {node_id: float('inf') for node_id in graph.nodes}
    prev = {node_id: None for node_id in graph.nodes}
    dist[start] = 0
    
    end_node = graph.nodes[end]
    
    def heuristic(node_id: int) -> float:
        node = graph.nodes[node_id]
        return haversine(node.lat, node.lon, end_node.lat, end_node.lon)
    
    pq = [(heuristic(start), start)]
    nodes_explored = 0
    visited = set()
    skipped = set() ## Track nodes skipped due to time window violations to they may be queued without being able to be relaxed
    
    while pq:
        _, u = heapq.heappop(pq)
        
        if u in visited:
            continue
        
        if u not in skipped:
            nodes_explored += 1 ## this value is for unique nodes expolred, so I'm not counting skipped nodes


        if u == end:
            break


        # Time window check
        #print(f"Checking node {u} at time {dist[u]:.2f}, window: [{graph.nodes[u].early}, {graph.nodes[u].late}]")
        if not (graph.nodes[u].early <= dist[u] <= graph.nodes[u].late):
            print(f"Time window constraint violated at node {u} at time {dist[u]:.2f}. Time window: [{graph.nodes[u].early}, {graph.nodes[u].late}].")
            if(dist[u] < graph.nodes[u].early): ## if the node was visited too early, it can be revisited later
                print(f"Node {u} might be reached later after time {graph.nodes[u].early}.")
                skipped.add(u)
            continue  # process the next node in the priority queue

        for edge in graph.adj_list.get(u, []):
            v = edge.to
            alt = dist[u] + edge.weight

            if alt < dist[v] or v in skipped: ## allow "relaxation" of skipped nodes even though in increases the distance because it is possible to reach their time window with a longer path
                dist[v] = alt
                prev[v] = u
                f_score = alt + heuristic(v)
                heapq.heappush(pq, (f_score, v))
        
        visited.add(u) ## added this to the bottom to make skipped nodes not show up in the visited set
    return dist, prev, nodes_explored


def astar_priority(graph: Graph, start: int, end_nodes: List[int]) -> Tuple[Dict[int, float], List[int], int]:
    """
    A* algorithm for shortest path visiting multiple nodes in sequence
    Returns: (distances, path as list, nodes explored)
    """
    total_nodes_explored = 0
    current_start = start
    full_path = [start]
    cumulative_distance = 0
    priorities_visited = set()
    
    for end in end_nodes: ## run A* for each end node in order
        if end in priorities_visited:
            continue  # Skip already visited priority nodes, shortening the path if a priority node was accidentally visited out of order

        ## Otherwise normal A* implementation
        dist = {node_id: float('inf') for node_id in graph.nodes}
        prev = {node_id: None for node_id in graph.nodes}
        dist[current_start] = 0
        end_node = graph.nodes[end]
        
        def heuristic(node_id: int) -> float:
            node = graph.nodes[node_id]
            return haversine(node.lat, node.lon, end_node.lat, end_node.lon)
        
        pq = [(heuristic(current_start), current_start)]
        nodes_explored = 0
        visited = set()
        
        while pq:
            _, u = heapq.heappop(pq)
            if u in visited:
                continue
            visited.add(u)
            if (u in end_nodes) and (u != end) and (u not in priorities_visited):
                print(f"Priority node {u} reached early (out of order). Will not make a seperate stop here.")
                priorities_visited.add(u)
            nodes_explored += 1
            if u == end:
                priorities_visited.add(u)
                break
            for edge in graph.adj_list.get(u, []):
                v = edge.to
                alt = dist[u] + edge.weight
                if alt < dist[v]:
                    dist[v] = alt
                    prev[v] = u
                    f_score = alt + heuristic(v)
                    heapq.heappush(pq, (f_score, v))

        ## Reconstruct this part of the path and append to full path
        
        segment = reconstruct_path(prev, current_start, end)
        if segment:
            full_path.extend(segment[1:])
        cumulative_distance += dist[end]
        total_nodes_explored += nodes_explored
        current_start = end
    
    combined_dist = {node: float('inf') for node in graph.nodes}
    combined_dist[end_nodes[-1]] = cumulative_distance
    
    return combined_dist, full_path, total_nodes_explored

def bellman_ford(graph: Graph, start: int, end: int) -> Tuple[Optional[Dict[int, float]], Optional[Dict[int, Optional[int]]], int]:
    """
    Bellman-Ford algorithm for shortest path
    Can handle negative weights and detect negative cycles
    Returns: (distances, previous nodes, nodes explored) or (None, None, 0) if negative cycle detected
    """
    dist = {node_id: float('inf') for node_id in graph.nodes}
    prev = {node_id: None for node_id in graph.nodes}
    dist[start] = 0
    
    nodes_explored = 0
    node_count = len(graph.nodes)
    
    # Relax edges |V| - 1 times
    for i in range(node_count - 1):
        updated = False
        for u in graph.nodes:
            if dist[u] == float('inf'):
                continue
            
            for edge in graph.adj_list.get(u, []):
                v = edge.to
                if dist[u] + edge.weight < dist[v]:
                    dist[v] = dist[u] + edge.weight
                    prev[v] = u
                    updated = True
        
        nodes_explored += 1
        if not updated:
            break
    
    # Check for negative cycles
    for u in graph.nodes:
        if dist[u] == float('inf'):
            continue
        
        for edge in graph.adj_list.get(u, []):
            v = edge.to
            if dist[u] + edge.weight < dist[v]:
                return None, None, 0  # Negative cycle detected
    
    return dist, prev, nodes_explored


def reconstruct_path(prev: Dict[int, Optional[int]], start: int, end: int) -> Optional[List[int]]:
    """Reconstruct path from start to end using previous nodes"""
    if prev[end] is None and start != end:
        return None
    
    path = []
    current = end
    while current is not None:
        path.append(current)
        current = prev[current]
    
    path.reverse()
    return path


def print_path(graph: Graph, prev: Dict[int, Optional[int]], start: int, end: int, distance: float):
    """Print the path from start to end"""
    path = reconstruct_path(prev, start, end)
    
    if path is None:
        print("No path found")
        return
    
    path_str = " -> ".join(str(node) for node in path)
    print(f"Path from {start} to {end}: {path_str}")
    print(f"Total distance: {distance:.2f} km")


def load_graph(nodes_file: str, edges_file: str) -> Graph:
    """Load graph from CSV files"""
    graph = Graph()
    
    # Load nodes
    with open(nodes_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            node_id = int(row['id'])
            lat = float(row['lat'])
            lon = float(row['lon'])
            early = int(row['earliest'])
            late = int(row['latest'])
            graph.add_node(node_id, lat, lon, early, late)
    
    # Load edges
    with open(edges_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            from_id = int(row['from'])
            to_id = int(row['to'])
            distance = float(row['distance'])
            graph.add_edge(from_id, to_id, distance)
    
    return graph


def load_end_nodes_from_csv(filename: str) -> List[int]:
    """Loads end nodes and priorities from a CSV file.
    Returns: A list of tuples, where each tuple is (node_id, priority).
    """
    end_nodes = []
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            node_id = int(row['node'])
            priority = float(row['priority'])
            end_nodes.append((node_id, priority))

    end_nodes.sort(key=lambda item: item[1])  # Sort by priority so I can get rid of the extra unncessary data
    '''
    for node in end_nodes:
        print(f"Node {node[0]} with priority {node[1]}")
    '''
    return [node_id for node_id, _ in end_nodes]


def main():
    if len(sys.argv) != 6:
        print(f"Usage: {sys.argv[0]} <nodes.csv> <edges.csv> <start_node> <end_node|priorities.csv> <algorithm>")
        print("Algorithms: dijkstra, astar, bellman-ford, astar_time")
        sys.exit(1)
    
    nodes_file = sys.argv[1]
    edges_file = sys.argv[2]
    start_node = int(sys.argv[3])
    end_node_arg = sys.argv[4]
    algorithm = sys.argv[5]
    
    # Load graph
    graph = load_graph(nodes_file, edges_file)


    if end_node_arg.isdigit():
        end_nodes = [int(end_node_arg)]  # Single end node
    else:
        end_nodes = [node_id for node_id in load_end_nodes_from_csv(end_node_arg)]  # Load from CSV

        # Validate nodes
    if start_node not in graph.nodes or end_nodes[0] not in graph.nodes:
        print("Invalid start or end node")
        sys.exit(1)
    
    # Run selected algorithm
    if algorithm == "dijkstra":
        print("=== Dijkstra's Algorithm ===")
        dist, prev, nodes_explored = dijkstra(graph, start_node, end_nodes[0])
    elif algorithm == "astar":
        print("=== A* Algorithm ===")
        dist, prev, nodes_explored = astar(graph, start_node, end_nodes[0])
    elif algorithm == "bellman-ford":
        print("=== Bellman-Ford Algorithm ===")
        dist, prev, nodes_explored = bellman_ford(graph, start_node, end_nodes[0])
        if dist is None:
            print("Negative cycle detected!")
            sys.exit(1)
    elif algorithm == "astar_time":
        print("=== A* Algorithm with Time Windows ===")
        dist, prev, nodes_explored = astar_time(graph, start_node, end_nodes[0])
        dist_astar, prev_astar, nodes_explored_astar = astar(graph, start_node, end_nodes[0]) ## running normal astar so I have someting to compare against

        if dist[end_nodes[0]] is None or dist[end_nodes[0]] == float('inf'): ## if no feasible path was found that fits the time windows
            print("No feasible path found due to time constraints.")
            print("!!! WARNING!!! Displaying the closest match path ignoring time constraints:")
            print_path(graph, prev_astar, start_node, end_nodes[0], dist_astar[end_nodes[0]])
        elif dist[end_nodes[0]] > dist_astar[end_nodes[0]]: ## if a feasible path was found but it is longer than the unconstrained shortest path
            print("A* with Time Windows found a path, but it was not the shortest possible path. The shortest path violated time constraints.")
            print("Displaying the feasible path found:")
    elif algorithm == "astar_priority":
        print("=== A* Algorithm with Note Priorities ===")
        dist, path, nodes_explored = astar_priority(graph, start_node, end_nodes)
        path_str = " -> ".join(str(node) for node in path)
        print(f"Path from {start_node} to {end_nodes[-1]}: {path_str}")
        print(f"Total distance: {dist[end_nodes[-1]]:.2f} km")
        print(f"Nodes explored: {nodes_explored}")
        sys.exit(0)

    else:
        print(f"Unknown algorithm: {algorithm}")
        print("Available algorithms: dijkstra, astar, bellman-ford")
        sys.exit(1)
    
    # Print results
    print_path(graph, prev, start_node, end_nodes[0], dist[end_nodes[0]])
    print(f"Nodes explored: {nodes_explored}")


if __name__ == "__main__":
    main()