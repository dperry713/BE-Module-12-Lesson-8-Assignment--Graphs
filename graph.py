class Graph:
    def __init__(self):
        self.vertices = {}

    def add_vertex(self, vertex):
        if vertex not in self.vertices:
            self.vertices[vertex] = {}

    def add_edge(self, source, destination, weight):
        if source in self.vertices and destination in self.vertices:
            self.vertices[source][destination] = weight
            self.vertices[destination][source] = weight  # For undirected graph

    def get_neighbors(self, vertex):
        return self.vertices.get(vertex, {})

import heapq

def dijkstra(graph, start):
    # Priority queue to store (distance, vertex)
    priority_queue = []
    heapq.heappush(priority_queue, (0, start))

    # Distances dictionary to store the shortest path distance to each vertex
    distances = {vertex: float('inf') for vertex in graph.vertices}
    distances[start] = 0

    # Predecessors to reconstruct the path
    predecessors = {vertex: None for vertex in graph.vertices}

    while priority_queue:
        current_distance, current_vertex = heapq.heappop(priority_queue)

        # Skip if the distance is not the shortest (queue may contain outdated entries)
        if current_distance > distances[current_vertex]:
            continue

        for neighbor, weight in graph.get_neighbors(current_vertex).items():
            distance = current_distance + weight

            # If a shorter path is found
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                predecessors[neighbor] = current_vertex
                heapq.heappush(priority_queue, (distance, neighbor))

    return distances, predecessors

# Create the graph
graph = Graph()
graph.add_vertex('A')
graph.add_vertex('B')
graph.add_vertex('C')
graph.add_vertex('D')
graph.add_edge('A', 'B', 5)
graph.add_edge('B', 'C', 3)
graph.add_edge('A', 'C', 10)
graph.add_edge('B', 'D', 2)
graph.add_edge('C', 'D', 1)

# Run Dijkstra's algorithm
source = 'A'
distances, predecessors = dijkstra(graph, source)

# Print results
print("Shortest Distances:", distances)
print("Predecessors:", predecessors)

# Output Expected:
# Shortest Distances: {'A': 0, 'B': 5, 'C': 8, 'D': 7}
# Predecessors: {'A': None, 'B': 'A', 'C': 'B', 'D': 'B'}

def reconstruct_path(predecessors, start, end):
    path = []
    current = end
    while current is not None:
        path.append(current)
        current = predecessors[current]
    return path[::-1]  # Reverse the path

# Example
print("Path from A to D:", reconstruct_path(predecessors, 'A', 'D'))  # Output: ['A', 'B', 'D']
