import numpy as np
import random

class Environment:
    """
    Represents the simulation environment, including the grid, obstacles, and boundary conditions.
    """
    
    def __init__(self, grid_size=(20, 20)):
        """
        Initialize the environment.
        
        Args:
            grid_size (tuple): Size of the grid as (width, height)
        """
        self.grid_size = grid_size
        self.grid = np.zeros(grid_size, dtype=int)  # 0 = free space, 1 = obstacle
        
    def add_obstacle(self, position):
        """
        Add an obstacle at the specified position.
        
        Args:
            position (tuple): Position (x, y) to place the obstacle
        """
        x, y = int(position[0]), int(position[1])
        if 0 <= x < self.grid_size[0] and 0 <= y < self.grid_size[1]:
            self.grid[x, y] = 1
            
    def remove_obstacle(self, position):
        """
        Remove an obstacle from the specified position.
        
        Args:
            position (tuple): Position (x, y) to remove the obstacle from
        """
        x, y = int(position[0]), int(position[1])
        if 0 <= x < self.grid_size[0] and 0 <= y < self.grid_size[1]:
            self.grid[x, y] = 0
            
    def clear_obstacles(self):
        """
        Remove all obstacles from the grid.
        """
        self.grid = np.zeros(self.grid_size, dtype=int)
        
    def is_valid_position(self, position):
        """
        Check if a position is valid (within bounds and not an obstacle).
        
        Args:
            position (tuple): Position (x, y) to check
            
        Returns:
            bool: True if position is valid, False otherwise
        """
        x, y = int(round(position[0])), int(round(position[1]))
        
        # Check if position is within bounds
        if x < 0 or x >= self.grid_size[0] or y < 0 or y >= self.grid_size[1]:
            return False
        
        # Check if position contains an obstacle
        if self.grid[x, y] == 1:
            return False
        
        return True
        
    def generate_random_obstacles(self, count=10, exclude=None):
        """
        Generate random obstacles on the grid.
        
        Args:
            count (int): Number of obstacles to generate
            exclude (list): List of positions to exclude from obstacle placement
        """
        if exclude is None:
            exclude = []
            
        # Convert exclude positions to grid coordinates
        exclude_positions = [(int(pos[0]), int(pos[1])) for pos in exclude]
        
        # Generate random obstacles
        for _ in range(count):
            x = random.randint(0, self.grid_size[0] - 1)
            y = random.randint(0, self.grid_size[1] - 1)
            
            # Skip if position is in exclude list
            if (x, y) in exclude_positions:
                continue
                
            self.add_obstacle((x, y))
            
    def load_map(self, map_data):
        """
        Load a map from a 2D array.
        
        Args:
            map_data (numpy.ndarray): 2D array representing the map
        """
        if map_data.shape == self.grid_size:
            self.grid = map_data.copy()
        else:
            raise ValueError(f"Map size {map_data.shape} does not match grid size {self.grid_size}")
            
    def save_map(self):
        """
        Save the current map.
        
        Returns:
            numpy.ndarray: 2D array representing the map
        """
        return self.grid.copy()
