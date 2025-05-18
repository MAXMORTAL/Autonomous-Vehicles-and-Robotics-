import numpy as np
import math

def simulate_lidar(position, heading, environment, num_rays=12, max_range=5):
    """
    Simulates a LiDAR sensor by casting rays in various directions and detecting obstacles.
    
    Args:
        position (tuple): Current position (x, y)
        heading (float): Current heading in degrees
        environment (Environment): The environment object containing obstacle information
        num_rays (int): Number of rays to cast
        max_range (float): Maximum detection range
        
    Returns:
        list: List of points where each ray ended (either hit an obstacle or reached max range)
    """
    points = []
    
    # Convert heading to radians
    heading_rad = math.radians(heading)
    
    # Cast rays in various angles around the vehicle
    for i in range(num_rays):
        # Calculate ray angle (spread rays evenly around the vehicle)
        angle = heading_rad + (i * (2 * math.pi / num_rays))
        
        # Initialize ray start point
        ray_x, ray_y = position
        
        # Step size for ray casting
        step_size = 0.1
        
        # Ray length
        ray_length = 0
        
        # Cast ray until it hits an obstacle or reaches max range
        while ray_length < max_range:
            # Move along the ray
            ray_x += step_size * math.cos(angle)
            ray_y += step_size * math.sin(angle)
            ray_length += step_size
            
            # Check if we hit an obstacle
            rounded_pos = (round(ray_x), round(ray_y))
            if not environment.is_valid_position(rounded_pos):
                break
        
        # Add the endpoint to the points list
        points.append((ray_x, ray_y))
    
    return points

def simulate_proximity_sensors(position, environment, num_sensors=4, max_range=2):
    """
    Simulates proximity sensors (like ultrasonic sensors) at fixed positions around the vehicle.
    
    Args:
        position (tuple): Current position (x, y)
        environment (Environment): The environment object containing obstacle information
        num_sensors (int): Number of sensors (4 = front, right, back, left)
        max_range (float): Maximum detection range
        
    Returns:
        list: List of distances detected by each sensor
    """
    distances = []
    
    # Sensor directions (in degrees)
    angles = [0, 90, 180, 270]  # Front, Right, Back, Left
    
    # Check each sensor
    for angle in angles[:num_sensors]:
        # Convert angle to radians
        angle_rad = math.radians(angle)
        
        # Initialize sensor start point
        sensor_x, sensor_y = position
        
        # Step size for detection
        step_size = 0.1
        
        # Distance measured
        distance = 0
        
        # Check until we hit an obstacle or reach max range
        while distance < max_range:
            # Move along the sensor direction
            sensor_x += step_size * math.cos(angle_rad)
            sensor_y += step_size * math.sin(angle_rad)
            distance += step_size
            
            # Check if we hit an obstacle
            rounded_pos = (round(sensor_x), round(sensor_y))
            if not environment.is_valid_position(rounded_pos):
                break
        
        # Add the distance to the list
        distances.append(distance)
    
    return distances
