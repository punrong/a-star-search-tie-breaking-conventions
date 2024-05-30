import heapq

# Define the heuristic function
def heuristic(node, goal):
    # Calculate the Manhattan distance as a heuristic
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

# A* search function
def astar_search(grid, start, goal):
    open_list = []  # Priority queue to store nodes to be explored
    closed_set = set()  # Set to store explored nodes
    came_from = {}  # Dictionary to store parent nodes for backtracking
    
    # Initialize G-values and F-values
    g_values = {start: 0}
    f_values = {start: heuristic(start, goal)}
    
    # Push the start node onto the open list
    heapq.heappush(open_list, (f_values[start], g_values[start], start))
    
    while open_list:
        _, _, current = heapq.heappop(open_list)  # Get the node with the lowest F-value and lowest G-value
        
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
        
        neighbors = get_neighbors(current, grid)
        
        # Sort neighbors by their G-values (tie-breaker)
        neighbors.sort(key=lambda n: g_values[n])
        
        for neighbor in neighbors:
            if neighbor in closed_set:
                continue  # Skip already explored nodes
            
            tentative_g = g_values[current] + 1  # Assuming unit cost
            
            if (
                neighbor not in [item[2] for item in open_list] or
                tentative_g < g_values[neighbor]
            ):
                # This path to the neighbor is better than any previous one
                came_from[neighbor] = current
                g_values[neighbor] = tentative_g
                h_value = heuristic(neighbor, goal)
                f_value = tentative_g + h_value
                
                if neighbor not in [item[2] for item in open_list]:
                    heapq.heappush(open_list, (f_value, g_values[neighbor], neighbor))
    
    return []  # No path found

# Function to get neighboring nodes
def get_neighbors(node, grid):
    neighbors = []
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Right, Down, Left, Up
    
    for direction in directions:
        neighbor = (node[0] + direction[0], node[1] + direction[1])
        
        if (
            0 <= neighbor[0] < len(grid) and
            0 <= neighbor[1] < len(grid[0]) and
            grid[neighbor[0]][neighbor[1]] != 1
        ):
            neighbors.append(neighbor)
    
    return neighbors

# Example grid
grid = [
    [0, 0, 0, 0, 0],
    [0, 1, 1, 0, 0],
    [0, 0, 0, 1, 0],
    [0, 1, 0, 0, 0],
    [0, 0, 0, 0, 0]
]

start = (0, 0)
goal = (4, 4)

# Find and print the optimal path
path = astar_search(grid, start, goal)
if path:
    print("Optimal Path:")
    for node in path:
        print(node)
else:
    print("No path found.")
