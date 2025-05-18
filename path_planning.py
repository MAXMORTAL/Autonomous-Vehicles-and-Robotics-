import numpy as np
import heapq
from collections import defaultdict

def a_star(start, goal, environment):
    """
    Implements the A* pathfinding algorithm to find the optimal path
    from start to goal.
    
    Args:
        start (tuple): Starting position (x, y)
        goal (tuple): Goal position (x, y)
        environment (Environment): The environment object containing obstacle information
        
    Returns:
        list: List of coordinates representing the path from start to goal,
              or an empty list if no path is found
    """
    # Round the positions to grid coordinates
    start = (round(start[0]), round(start[1]))
    goal = (round(goal[0]), round(goal[1]))
    
    # Define heuristic function (Euclidean distance)
    def heuristic(a, b):
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
    
    # Define possible movement directions (8-directional movement)
    directions = [
        (0, 1),   # Up
        (1, 0),   # Right
        (0, -1),  # Down
        (-1, 0),  # Left
        (1, 1),   # Up-Right
        (-1, 1),  # Up-Left
        (1, -1),  # Down-Right
        (-1, -1)  # Down-Left
    ]
    
    # Initialize open and closed sets
    open_set = []
    closed_set = set()
    
    # Dictionary to store g scores (cost from start to current node)
    g_score = defaultdict(lambda: float('inf'))
    g_score[start] = 0
    
    # Dictionary to store f scores (g_score + heuristic)
    f_score = defaultdict(lambda: float('inf'))
    f_score[start] = heuristic(start, goal)
    
    # Dictionary to store the parent of each node
    came_from = {}
    
    # Add starting node to open set
    heapq.heappush(open_set, (f_score[start], start))
    
    while open_set:
        # Get the node with the lowest f_score
        current_f, current = heapq.heappop(open_set)
        
        # If we've reached the goal, reconstruct and return the path
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path
        
        # Add current node to closed set
        closed_set.add(current)
        
        # Check all neighboring nodes
        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            
            # Skip if neighbor is in closed set
            if neighbor in closed_set:
                continue
            
            # Skip if neighbor is out of bounds or an obstacle
            if not environment.is_valid_position(neighbor):
                continue
            
            # Calculate tentative g score
            tentative_g = g_score[current]
            
            # Diagonal movement costs more
            if dx != 0 and dy != 0:
                tentative_g += 1.414  # sqrt(2)
            else:
                tentative_g += 1
            
            # If this path is better than any previous one
            if tentative_g < g_score[neighbor]:
                # Record this path
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                
                # Add to open set if not already there
                if neighbor not in [i[1] for i in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    # If we get here, no path was found
    return []

def dijkstra(start, goal, environment):
    """
    Implements Dijkstra's algorithm to find the shortest path from start to goal.
    
    Args:
        start (tuple): Starting position (x, y)
        goal (tuple): Goal position (x, y)
        environment (Environment): The environment object containing obstacle information
        
    Returns:
        list: List of coordinates representing the path from start to goal,
              or an empty list if no path is found
    """
    # Round the positions to grid coordinates
    start = (round(start[0]), round(start[1]))
    goal = (round(goal[0]), round(goal[1]))
    
    # Define possible movement directions (8-directional movement)
    directions = [
        (0, 1),   # Up
        (1, 0),   # Right
        (0, -1),  # Down
        (-1, 0),  # Left
        (1, 1),   # Up-Right
        (-1, 1),  # Up-Left
        (1, -1),  # Down-Right
        (-1, -1)  # Down-Left
    ]
    
    # Initialize priority queue, visited set, and distance dictionary
    queue = [(0, start)]  # (distance, node)
    visited = set()
    distance = {start: 0}
    previous = {}
    
    while queue:
        # Get the node with the smallest distance
        current_dist, current = heapq.heappop(queue)
        
        # If we've reached the goal, reconstruct and return the path
        if current == goal:
            path = []
            while current in previous:
                path.append(current)
                current = previous[current]
            path.append(start)
            path.reverse()
            return path
        
        # Skip if we've already processed this node
        if current in visited:
            continue
            
        # Mark as visited
        visited.add(current)
        
        # Check all neighboring nodes
        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            
            # Skip if neighbor is out of bounds or an obstacle
            if not environment.is_valid_position(neighbor):
                continue
                
            # Calculate new distance to neighbor
            # Diagonal movement costs more
            if dx != 0 and dy != 0:
                weight = 1.414  # sqrt(2)
            else:
                weight = 1
                
            new_dist = current_dist + weight
            
            # If this is a new node or we found a shorter path
            if neighbor not in distance or new_dist < distance[neighbor]:
                distance[neighbor] = new_dist
                previous[neighbor] = current
                heapq.heappush(queue, (new_dist, neighbor))
    
    # If we get here, no path was found
    return []
