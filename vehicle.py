import numpy as np
import math

class Vehicle:
    """
    Represents a vehicle/robot in the simulation with position, heading, and movement capabilities.
    """
    
    def __init__(self, position, heading=0, velocity=0, environment=None):
        """
        Initialize the vehicle.
        
        Args:
            position (tuple): Starting position (x, y)
            heading (float): Initial heading in degrees (0 = right, 90 = up)
            velocity (float): Initial velocity
            environment (Environment): Reference to the environment
        """
        self.position = position
        self.heading = heading
        self.velocity = velocity
        self.environment = environment
        self.path_index = 0  # For following a planned path
        
    def move(self, distance):
        """
        Move the vehicle in the current heading direction.
        
        Args:
            distance (float): Distance to move
            
        Returns:
            bool: True if the move was successful, False if blocked
        """
        # Calculate new position
        heading_rad = math.radians(self.heading)
        new_x = self.position[0] + distance * math.cos(heading_rad)
        new_y = self.position[1] + distance * math.sin(heading_rad)
        
        # Check if new position is valid
        if self.environment and self.environment.is_valid_position((new_x, new_y)):
            self.position = (new_x, new_y)
            return True
        
        return False
        
    def turn(self, angle):
        """
        Turn the vehicle by the specified angle.
        
        Args:
            angle (float): Angle to turn in degrees (positive = counterclockwise)
        """
        self.heading = (self.heading + angle) % 360
        
    def follow_path(self, path):
        """
        Follow a pre-planned path.
        
        Args:
            path (list): List of positions to follow
            
        Returns:
            bool: True if reached the end of the path, False otherwise
        """
        if not path or self.path_index >= len(path):
            return True
        
        # Get the next target position
        target = path[self.path_index]
        
        # Calculate direction and distance to target
        dx = target[0] - self.position[0]
        dy = target[1] - self.position[1]
        distance = math.sqrt(dx**2 + dy**2)
        
        # If we're close enough to the target, move to the next point
        if distance < 0.2:
            self.path_index += 1
            if self.path_index >= len(path):
                return True
            return False
        
        # Calculate the angle to the target
        target_angle = math.degrees(math.atan2(dy, dx)) % 360
        
        # Find the shortest way to turn
        angle_diff = (target_angle - self.heading) % 360
        if angle_diff > 180:
            angle_diff -= 360
        
        # Turn towards the target (with a maximum turn rate)
        max_turn_rate = 15  # degrees per step
        turn_amount = max(-max_turn_rate, min(max_turn_rate, angle_diff))
        self.turn(turn_amount)
        
        # Move towards the target
        move_speed = 0.2  # units per step
        self.velocity = move_speed
        self.move(move_speed)
        
        return False
    
    def get_sensor_positions(self):
        """
        Return positions for sensors on the vehicle.
        
        Returns:
            list: List of (x, y) positions for sensors
        """
        heading_rad = math.radians(self.heading)
        
        # Position sensors relative to vehicle center
        sensor_positions = []
        
        # Front sensor
        front_x = self.position[0] + 0.5 * math.cos(heading_rad)
        front_y = self.position[1] + 0.5 * math.sin(heading_rad)
        sensor_positions.append((front_x, front_y))
        
        # Right sensor
        right_angle = heading_rad + math.pi/2
        right_x = self.position[0] + 0.5 * math.cos(right_angle)
        right_y = self.position[1] + 0.5 * math.sin(right_angle)
        sensor_positions.append((right_x, right_y))
        
        # Rear sensor
        rear_angle = heading_rad + math.pi
        rear_x = self.position[0] + 0.5 * math.cos(rear_angle)
        rear_y = self.position[1] + 0.5 * math.sin(rear_angle)
        sensor_positions.append((rear_x, rear_y))
        
        # Left sensor
        left_angle = heading_rad - math.pi/2
        left_x = self.position[0] + 0.5 * math.cos(left_angle)
        left_y = self.position[1] + 0.5 * math.sin(left_angle)
        sensor_positions.append((left_x, left_y))
        
        return sensor_positions
