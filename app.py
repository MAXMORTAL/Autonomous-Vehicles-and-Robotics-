import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
import time
import pandas as pd
from matplotlib.patches import Circle, Wedge
from matplotlib.collections import PatchCollection
import plotly.graph_objects as go

from utils.path_planning import a_star, dijkstra
from utils.sensors import simulate_lidar, simulate_proximity_sensors
from utils.vehicle import Vehicle
from utils.environment import Environment

# Page configuration
st.set_page_config(
    page_title="Autonomous Vehicle Simulation",
    page_icon="🤖",
    layout="wide"
)

# Initialize session state variables
if 'environment' not in st.session_state:
    st.session_state.environment = Environment(grid_size=(20, 20))
    
if 'vehicle' not in st.session_state:
    start_position = (1, 1)
    st.session_state.vehicle = Vehicle(start_position, 
                                       heading=0,
                                       environment=st.session_state.environment)

if 'goal' not in st.session_state:
    st.session_state.goal = (18, 18)

if 'path' not in st.session_state:
    st.session_state.path = []
    
if 'is_running' not in st.session_state:
    st.session_state.is_running = False
    
if 'control_mode' not in st.session_state:
    st.session_state.control_mode = "Autonomous"
    
if 'algorithm' not in st.session_state:
    st.session_state.algorithm = "A*"
    
if 'telemetry' not in st.session_state:
    st.session_state.telemetry = pd.DataFrame(columns=['Time', 'X', 'Y', 'Heading', 'Velocity'])
    
if 'time_elapsed' not in st.session_state:
    st.session_state.time_elapsed = 0

# Main title
st.title("🤖 Autonomous Vehicle Simulation")

# Split the screen into a main area and a sidebar
main_col1, main_col2 = st.columns([3, 2])

with main_col1:
    # Simulation visualization
    st.subheader("Simulation Environment")
    
    # Plotly for interactive visualization
    fig = go.Figure()
    
    # Draw grid
    grid_x, grid_y = st.session_state.environment.grid_size
    
    # Draw obstacles
    obstacle_x = []
    obstacle_y = []
    for x in range(grid_x):
        for y in range(grid_y):
            if st.session_state.environment.grid[x, y] == 1:  # 1 represents obstacle
                obstacle_x.append(x)
                obstacle_y.append(y)
    
    if obstacle_x:
        fig.add_trace(go.Scatter(
            x=obstacle_x, 
            y=obstacle_y,
            mode='markers',
            marker=dict(
                symbol='square',
                size=15,
                color='gray',
            ),
            name='Obstacles'
        ))
    
    # Draw vehicle position
    vehicle_x, vehicle_y = st.session_state.vehicle.position
    
    # Calculate endpoints of heading indicator
    heading_length = 0.5
    heading_x = vehicle_x + heading_length * np.cos(np.radians(st.session_state.vehicle.heading))
    heading_y = vehicle_y + heading_length * np.sin(np.radians(st.session_state.vehicle.heading))
    
    fig.add_trace(go.Scatter(
        x=[vehicle_x], 
        y=[vehicle_y],
        mode='markers',
        marker=dict(
            symbol='circle',
            size=15,
            color='blue',
        ),
        name='Vehicle'
    ))
    
    # Draw heading indicator
    fig.add_trace(go.Scatter(
        x=[vehicle_x, heading_x], 
        y=[vehicle_y, heading_y],
        mode='lines',
        line=dict(color='blue', width=2),
        showlegend=False
    ))
    
    # Draw goal
    goal_x, goal_y = st.session_state.goal
    fig.add_trace(go.Scatter(
        x=[goal_x], 
        y=[goal_y],
        mode='markers',
        marker=dict(
            symbol='star',
            size=15,
            color='green',
        ),
        name='Goal'
    ))
    
    # Draw path if it exists
    if st.session_state.path:
        path_x = [p[0] for p in st.session_state.path]
        path_y = [p[1] for p in st.session_state.path]
        fig.add_trace(go.Scatter(
            x=path_x, 
            y=path_y,
            mode='lines',
            line=dict(color='orange', width=2),
            name='Planned Path'
        ))
    
    # Draw sensor readings
    if st.session_state.control_mode == "Autonomous":
        # Simulate LiDAR
        lidar_points = simulate_lidar(st.session_state.vehicle.position, 
                                    st.session_state.vehicle.heading, 
                                    st.session_state.environment,
                                    num_rays=12, 
                                    max_range=5)
        
        if lidar_points:
            lidar_x = [p[0] for p in lidar_points]
            lidar_y = [p[1] for p in lidar_points]
            
            # Draw LiDAR rays
            for i in range(len(lidar_points)):
                fig.add_trace(go.Scatter(
                    x=[vehicle_x, lidar_x[i]], 
                    y=[vehicle_y, lidar_y[i]],
                    mode='lines',
                    line=dict(color='rgba(255, 0, 0, 0.3)', width=1),
                    showlegend=False if i > 0 else True,
                    name='LiDAR' if i == 0 else None
                ))
    
    # Set layout
    fig.update_layout(
        xaxis=dict(range=[-1, grid_x + 1], title='X'),
        yaxis=dict(range=[-1, grid_y + 1], title='Y', scaleanchor="x", scaleratio=1),
        margin=dict(l=20, r=20, t=20, b=20),
        legend=dict(x=0, y=1),
        template="plotly_white"
    )
    
    # Display the plot
    st.plotly_chart(fig, use_container_width=True)
    
    # Control buttons in a row
    control_col1, control_col2, control_col3 = st.columns(3)
    
    with control_col1:
        if st.button("Start/Resume" if not st.session_state.is_running else "Pause"):
            st.session_state.is_running = not st.session_state.is_running
            
            # If starting, plan a path
            if st.session_state.is_running and st.session_state.control_mode == "Autonomous":
                # Plan path based on selected algorithm
                if st.session_state.algorithm == "A*":
                    st.session_state.path = a_star(
                        st.session_state.vehicle.position, 
                        st.session_state.goal, 
                        st.session_state.environment
                    )
                else:  # Dijkstra
                    st.session_state.path = dijkstra(
                        st.session_state.vehicle.position, 
                        st.session_state.goal, 
                        st.session_state.environment
                    )
                
                # Reset vehicle attributes for new run
                st.session_state.vehicle.path_index = 0
                st.session_state.telemetry = pd.DataFrame(columns=['Time', 'X', 'Y', 'Heading', 'Velocity'])
                st.session_state.time_elapsed = 0
                
    with control_col2:
        if st.button("Reset"):
            # Reset vehicle to starting position
            st.session_state.vehicle.position = (1, 1)
            st.session_state.vehicle.heading = 0
            st.session_state.vehicle.velocity = 0
            st.session_state.vehicle.path_index = 0
            st.session_state.is_running = False
            st.session_state.path = []
            st.session_state.telemetry = pd.DataFrame(columns=['Time', 'X', 'Y', 'Heading', 'Velocity'])
            st.session_state.time_elapsed = 0
            
    with control_col3:
        if st.button("Clear Obstacles"):
            st.session_state.environment.clear_obstacles()
            st.session_state.path = []
            st.rerun()
            
    # Manual controls for the vehicle if in manual mode
    if st.session_state.control_mode == "Manual":
        st.write("Manual Controls:")
        manual_col1, manual_col2, manual_col3 = st.columns(3)
        
        with manual_col1:
            if st.button("⬆️"):
                new_x = vehicle_x + 0.5 * np.cos(np.radians(st.session_state.vehicle.heading))
                new_y = vehicle_y + 0.5 * np.sin(np.radians(st.session_state.vehicle.heading))
                
                # Check if the new position is valid
                if st.session_state.environment.is_valid_position((new_x, new_y)):
                    st.session_state.vehicle.position = (new_x, new_y)
                    
                    # Record telemetry
                    new_row = pd.DataFrame({
                        'Time': [st.session_state.time_elapsed],
                        'X': [new_x],
                        'Y': [new_y],
                        'Heading': [st.session_state.vehicle.heading],
                        'Velocity': [0.5]
                    })
                    st.session_state.telemetry = pd.concat([st.session_state.telemetry, new_row], ignore_index=True)
                    st.session_state.time_elapsed += 1
                    
                st.rerun()
        
        manual_col1, manual_col2, manual_col3 = st.columns(3)
        with manual_col1:
            if st.button("⬅️"):
                # Turn left
                st.session_state.vehicle.heading = (st.session_state.vehicle.heading + 15) % 360
                st.rerun()
                
        with manual_col2:
            if st.button("⬇️"):
                new_x = vehicle_x - 0.5 * np.cos(np.radians(st.session_state.vehicle.heading))
                new_y = vehicle_y - 0.5 * np.sin(np.radians(st.session_state.vehicle.heading))
                
                # Check if the new position is valid
                if st.session_state.environment.is_valid_position((new_x, new_y)):
                    st.session_state.vehicle.position = (new_x, new_y)
                    
                    # Record telemetry
                    new_row = pd.DataFrame({
                        'Time': [st.session_state.time_elapsed],
                        'X': [new_x],
                        'Y': [new_y],
                        'Heading': [st.session_state.vehicle.heading],
                        'Velocity': [-0.5]
                    })
                    st.session_state.telemetry = pd.concat([st.session_state.telemetry, new_row], ignore_index=True)
                    st.session_state.time_elapsed += 1
                    
                st.rerun()
                
        with manual_col3:
            if st.button("➡️"):
                # Turn right
                st.session_state.vehicle.heading = (st.session_state.vehicle.heading - 15) % 360
                st.rerun()
    
    # Update simulation if running (for autonomous mode)
    if st.session_state.is_running and st.session_state.control_mode == "Autonomous":
        # Move vehicle along the path
        reached_goal = st.session_state.vehicle.follow_path(st.session_state.path)
        
        # Record telemetry
        new_row = pd.DataFrame({
            'Time': [st.session_state.time_elapsed],
            'X': [st.session_state.vehicle.position[0]],
            'Y': [st.session_state.vehicle.position[1]],
            'Heading': [st.session_state.vehicle.heading],
            'Velocity': [st.session_state.vehicle.velocity]
        })
        st.session_state.telemetry = pd.concat([st.session_state.telemetry, new_row], ignore_index=True)
        st.session_state.time_elapsed += 1
        
        # Stop if reached goal
        if reached_goal:
            st.session_state.is_running = False
            st.success("Goal reached!")
        
        # Add a small delay to control simulation speed
        time.sleep(0.2)
        st.rerun()

with main_col2:
    # Sidebar controls
    with st.expander("Environment Settings", expanded=True):
        # Goal position setter
        st.subheader("Set Goal Position")
        goal_col1, goal_col2 = st.columns(2)
        with goal_col1:
            goal_x = st.number_input("Goal X", min_value=0, max_value=st.session_state.environment.grid_size[0]-1, value=st.session_state.goal[0])
        with goal_col2:
            goal_y = st.number_input("Goal Y", min_value=0, max_value=st.session_state.environment.grid_size[1]-1, value=st.session_state.goal[1])
        
        # Update goal position
        if (goal_x, goal_y) != st.session_state.goal:
            st.session_state.goal = (goal_x, goal_y)
            # Clear existing path when goal changes
            st.session_state.path = []
        
        # Add obstacles
        st.subheader("Add Obstacles")
        obstacle_col1, obstacle_col2, obstacle_col3 = st.columns(3)
        with obstacle_col1:
            obs_x = st.number_input("Obstacle X", min_value=0, max_value=st.session_state.environment.grid_size[0]-1, value=5)
        with obstacle_col2:
            obs_y = st.number_input("Obstacle Y", min_value=0, max_value=st.session_state.environment.grid_size[1]-1, value=5)
        with obstacle_col3:
            if st.button("Add Obstacle"):
                # Don't add obstacles at vehicle or goal positions
                if (obs_x, obs_y) != st.session_state.vehicle.position and (obs_x, obs_y) != st.session_state.goal:
                    st.session_state.environment.add_obstacle((obs_x, obs_y))
                    # Clear existing path when obstacles change
                    st.session_state.path = []
                    st.rerun()
        
        # Generate random obstacles
        if st.button("Generate Random Obstacles"):
            st.session_state.environment.clear_obstacles()
            st.session_state.environment.generate_random_obstacles(
                count=20, 
                exclude=[st.session_state.vehicle.position, st.session_state.goal]
            )
            # Clear existing path when obstacles change
            st.session_state.path = []
            st.rerun()
    
    # Control Mode and Algorithm Selection
    with st.expander("Simulation Settings", expanded=True):
        st.subheader("Control Settings")
        
        # Control mode selector
        control_mode = st.radio("Control Mode", ["Autonomous", "Manual"], index=0 if st.session_state.control_mode == "Autonomous" else 1)
        if control_mode != st.session_state.control_mode:
            st.session_state.control_mode = control_mode
            st.session_state.is_running = False
            st.session_state.path = []
            st.rerun()
        
        # Algorithm selector (only for autonomous mode)
        if st.session_state.control_mode == "Autonomous":
            algorithm = st.radio("Pathfinding Algorithm", ["A*", "Dijkstra"], index=0 if st.session_state.algorithm == "A*" else 1)
            if algorithm != st.session_state.algorithm:
                st.session_state.algorithm = algorithm
                st.session_state.path = []
        
        # Explanation of the selected algorithm
        if st.session_state.control_mode == "Autonomous":
            if st.session_state.algorithm == "A*":
                st.info("""
                **A* Algorithm**: A popular pathfinding algorithm that uses a heuristic to estimate the 
                distance to the goal. It's efficient and will find the shortest path.
                """)
            else:
                st.info("""
                **Dijkstra's Algorithm**: A graph search algorithm that finds the shortest path from a 
                starting node to all other nodes. It's slower than A* but guarantees the optimal path.
                """)
    
    # Telemetry Display
    with st.expander("Telemetry", expanded=True):
        st.subheader("Vehicle Telemetry")
        
        if not st.session_state.telemetry.empty:
            # Display current stats
            telemetry_col1, telemetry_col2 = st.columns(2)
            
            with telemetry_col1:
                st.metric("X Position", f"{st.session_state.vehicle.position[0]:.2f}")
                st.metric("Heading (degrees)", f"{st.session_state.vehicle.heading:.1f}")
                
            with telemetry_col2:
                st.metric("Y Position", f"{st.session_state.vehicle.position[1]:.2f}")
                st.metric("Velocity", f"{st.session_state.vehicle.velocity:.2f}")
            
            # Telemetry plot
            st.subheader("Position History")
            
            # Create a trajectory plot
            fig_telemetry = go.Figure()
            
            # Plot X and Y positions over time
            fig_telemetry.add_trace(go.Scatter(
                x=st.session_state.telemetry['Time'],
                y=st.session_state.telemetry['X'],
                mode='lines+markers',
                name='X Position'
            ))
            
            fig_telemetry.add_trace(go.Scatter(
                x=st.session_state.telemetry['Time'],
                y=st.session_state.telemetry['Y'],
                mode='lines+markers',
                name='Y Position'
            ))
            
            fig_telemetry.update_layout(
                xaxis_title='Time Steps',
                yaxis_title='Position',
                legend_title='Measurement',
                template='plotly_white'
            )
            
            st.plotly_chart(fig_telemetry, use_container_width=True)
        else:
            st.info("Telemetry data will appear once the simulation starts running.")
    
    # Sensor Visualization
    with st.expander("Sensor Data", expanded=True):
        st.subheader("Sensor Readings")
        
        if st.session_state.control_mode == "Autonomous":
            # Simulate LiDAR
            lidar_points = simulate_lidar(st.session_state.vehicle.position, 
                                      st.session_state.vehicle.heading, 
                                      st.session_state.environment,
                                      num_rays=12, 
                                      max_range=5)
            
            # Calculate distances
            if lidar_points:
                distances = []
                for point in lidar_points:
                    distance = np.sqrt((st.session_state.vehicle.position[0] - point[0])**2 + 
                                      (st.session_state.vehicle.position[1] - point[1])**2)
                    distances.append(distance)
                
                # Create a radar plot for LiDAR readings
                angles = np.linspace(0, 2*np.pi, len(distances), endpoint=False)
                
                # Close the plot by appending the first value to the end
                angles = np.append(angles, angles[0])
                distances = np.append(distances, distances[0])
                
                fig_radar = go.Figure()
                
                fig_radar.add_trace(go.Scatterpolar(
                    r=distances,
                    theta=np.degrees(angles),
                    fill='toself',
                    name='LiDAR Distance'
                ))
                
                fig_radar.update_layout(
                    polar=dict(
                        radialaxis=dict(
                            visible=True,
                            range=[0, 5]
                        )
                    ),
                    showlegend=False
                )
                
                st.plotly_chart(fig_radar, use_container_width=True)
                
                # Display obstacle detection info
                min_distance = min(distances)
                if min_distance < 1.5:
                    st.warning(f"⚠️ Obstacle detected at {min_distance:.2f} units away!")
                else:
                    st.success("✅ Path clear")
            else:
                st.info("No sensor data available yet. Start the simulation.")
        else:
            st.info("Sensor visualization is only available in Autonomous mode.")

    # Educational explanation
    with st.expander("Learning Resources", expanded=True):
        st.subheader("About This Simulation")
        
        st.markdown("""
        This educational tool demonstrates core concepts in autonomous vehicle and robotics navigation:
        
        - **Path Planning**: Visualize how A* and Dijkstra's algorithms find optimal paths through environments
        - **Obstacle Avoidance**: See how vehicles detect and navigate around obstacles
        - **Sensor Simulation**: Understand how LiDAR and proximity sensors help vehicles perceive their environment
        - **Control Systems**: Compare autonomous algorithmic control with manual human control
        
        Experiment with different obstacles, algorithms, and control modes to understand the challenges in autonomous navigation.
        """)

# Update the simulation at regular intervals if it's running
if st.session_state.is_running:
    time.sleep(0.1)  # Small delay to control the simulation speed
    st.rerun()
