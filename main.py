# Simulated Environment for Obstacle Detection and Response
# Dependencies: Install 'numpy' and 'opencv-python'

import numpy as np
import cv2
import time
import random

class SensorModule:
    """Simulates LiDAR-like distance sensing"""
    def get_obstacle_distance(self):
        # Random distance in meters; simulate obstacle at random intervals
        return random.uniform(0.5, 10.0)

class CameraModule:
    """Simulates object detection using a placeholder frame"""
    def detect_objects(self):
        print("Camera: Detecting objects ahead...")
        time.sleep(0.5)
        # Simulate detection of objects, with random detection probability
        if random.random() < 0.3:
            # 30% chance to detect a car or pedestrian
            detected_objects = ["car", "pedestrian"]
        else:
            detected_objects = []
        return detected_objects

class ControlModule:
    """Handles movement control decisions"""
    def __init__(self):
        self.speed = 0

    def accelerate(self):
        self.speed += 10
        print(f"Accelerating. Current speed: {self.speed} km/h")

    def brake(self):
        self.speed = max(0, self.speed - 20)
        print(f"Braking. Current speed: {self.speed} km/h")

    def steer(self, direction):
        print(f"Steering to the {direction}.")

    def stop(self):
        self.speed = 0
        print("Emergency stop! Vehicle halted.")

class AutonomousSystem:
    def __init__(self):
        self.sensor = SensorModule()

