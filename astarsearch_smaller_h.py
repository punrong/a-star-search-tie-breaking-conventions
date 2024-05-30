import heapq

class Node:
    def __init__(self, state, g_cost, h_cost, parent=None):
        self.state = state
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.parent = parent

    def __lt__(self, other):
        # Compare nodes based on their f-values (g_cost + h_cost)
        if self.g_cost + self.h_cost == other.g_cost + other.h_cost:
            # If f-values are equal, compare based on h-values
            return self.h_cost < other.h_cost
        return self.g_cost + self.h_cost < other.g_cost + other.h_cost

def astar_search(graph, start, goal):
    open_list = []  # Priority queue to store nodes to be explored
    closed_set = set()  # Set to store explored nodes
    came_from = {}  # Dictionary to store parent nodes for backtracking
    
    # Initialize G-values and F-values
    g_values = {start: 0}
    f_values = {start: heuristic(start, goal)}
    
    # Push the start node onto the open list
    heapq.heappush(open_list, (f_values[start], Node(start, 0, heuristic(start, goal))))
    
    while open_list:
        _, current_node = heapq.heappop(open_list)  # Get the node with the lowest F-value
        current = current_node.state
        
        if current == goal:
            # Backtrack to find the optimal path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path
        
        closed_set.add(current)  # Mark the current node as explored
        
        neighbors = get_neighbors(current, graph)
        
        # Sort neighbors by their H-values (tie-breaker)
        neighbors.sort(key=lambda n: heuristic(n, goal))
        
        for neighbor in neighbors:
            if neighbor in closed_set:
                continue  # Skip already explored nodes
            
            tentative_g = g_values[current] + 1  # Assuming unit cost
            
            if (
                neighbor not in [item[1].state for item in open_list] or
                tentative_g < g_values[neighbor]
            ):
                # This path to the neighbor is better than any previous one
                came_from[neighbor] = current
                g_values[neighbor] = tentative_g
                h_value = heuristic(neighbor, goal)
                f_value = tentative_g + h_value
                
                if neighbor not in [item[1].state for item in open_list]:
                    heapq.heappush(open_list, (f_value, Node(neighbor, tentative_g, h_value, current_node)))
    
    return []  # No path found.

# Function to get neighboring nodes based on the graph
def get_neighbors(node, graph):
    return graph.get(node, [])

# Define the heuristic function
def heuristic(node, goal):
    # Calculate the Manhattan distance as a heuristic
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

# Example graph represented as an adjacency dictionary
graph = {
    (0, 0): [(0, 1), (1, 0)],
    (0, 1): [(0, 0), (0, 2)],
    (0, 2): [(0, 1), (1, 2)],
    (1, 0): [(0, 0), (1, 1), (2, 0)],
    (1, 1): [(1, 0), (1, 2)],
    (1, 2): [(0, 2), (1, 1)],
    (2, 0): [(1, 0), (2, 1)],
    (2, 1): [(2, 0), (2, 2)],
    (2, 2): [(2, 1)]
}

start = (0, 0)
goal = (2, 2)

# Find and print the optimal path
path = astar_search(graph, start, goal)
if path:
    print("Optimal Path:")
    for node in path:
        print(node)
else:
    print("No path found.")
