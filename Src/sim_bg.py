#!/usr/bin/env python3

import pygame
import networkx as nx
import time
import heapq
import socket

# Initialize Pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Multi-Agent Pathfinding with Tasks")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)

# Define graph using networkx
graph = nx.Graph()

# Add nodes and edges
nodes = ["I1", "I2", "I3", "I4", "I5", "I6", "I7", "I8", "I9", "I10", "I11",
         "R1", "R2", "R3", "R4", "R5", "R6", "R7", "R8", "X1", "X2", "X3", "X4", "X5", "X6"]

edges = [
    ("I1", "I2"), ("I1", "R1"), ("I2", "I3"), ("I2", "I4"),
    ("I3", "R2"), ("I3", "X2"), ("I4", "I5"), ("I4", "R3"),
    ("I5", "I6"), ("I5", "I7"), ("I5", "I8"), ("I6", "R5"),
    ("I6", "X3"), ("I7", "R4"), ("I7", "X4"), ("I8", "I9"),
    ("I8", "R6"), ("I9", "I10"), ("I9", "I11"), ("I10", "R7"),
    ("I10", "X5"), ("I11", "R8"), ("I11", "X6"), ("X1", "I1")
]

graph.add_nodes_from(nodes)
graph.add_edges_from(edges)

# Node positions for visualization
positions = {
    "I1": (50, 100), "I2": (50, 150), "I3": (50, 250),
    "I4": (150, 150), "I5": (250, 150), "I6": (250, 100),
    "I7": (250, 250), "I8": (300, 150), "I9": (400, 150),
    "I10": (400, 100), "I11": (400, 250),
    "R1": (100, 100), "R2": (100, 250), "R3": (150, 200),
    "R4": (200, 250), "R5": (200, 100), "R6": (300, 200),
    "R7": (350, 100), "R8": (350, 250),
    "X1": (50, 50), "X2": (50, 300), "X3": (250, 50),
    "X4": (250, 300), "X5": (400, 50), "X6": (400, 300),
}

# Font for rendering text
font = pygame.font.Font(None, 24)

def draw_graph():
    screen.fill(WHITE)
    for edge in graph.edges:
        x1, y1 = positions[edge[0]]
        x2, y2 = positions[edge[1]]
        pygame.draw.line(screen, BLACK, (x1, y1), (x2, y2), 2)
    for node, (x, y) in positions.items():
        if node.startswith("X"):  # Depots
            pygame.draw.circle(screen, GREEN, (x, y), 20)
        else:  # Other nodes with black border and white fill
            pygame.draw.circle(screen, WHITE, (x, y), 20)  # White fill
            pygame.draw.circle(screen, BLACK, (x, y), 20, 2)  # Black border
        
        # Add the node label
        label = font.render(node, True, BLACK)
        label_rect = label.get_rect(center=(x, y))  # Center the text inside the circle
        screen.blit(label, label_rect)

# Pathfinding using networkx's shortest path
def find_path(start, goal):
    priority_queue = [(0, start, [])]
    visited = {}

    while priority_queue:
        current_cost, current_node, path = heapq.heappop(priority_queue)

        if current_node in visited and visited[current_node] <= current_cost:
            continue

        visited[current_node] = current_cost

        path = path + [current_node]

        if current_node == goal:
            return path

        for neighbor in graph.neighbors(current_node):
            weight = graph[current_node][neighbor].get('weight', 1)
            if neighbor not in visited or current_cost + weight < visited[neighbor]:
                heapq.heappush(priority_queue, (current_cost + weight, neighbor, path))

    return None
    return nx.shortest_path(graph, start, goal)

# Robots Attribute definition
robots = [
    {"id": 1, "color": RED, "path": [], "step": 0, "goal": None, "status": "free", "next_node": None, "curr_node": "X1"},
    {"id": 2, "color": BLUE, "path": [], "step": 0, "goal": None, "status": "free", "next_node": None, "curr_node": "X5"},
]

# Tasks: Input as start -> goal pairs
# tasks = [("X1", "X4"), ("X2", "X6"), ("X4", "X3"), ("X6", "X1"), ("X3", "X6")] # Set 1 of tasks success
# tasks = [("X1", "X4"), ("X5", "X2")] # Set 2 of tasks
tasks = [("X1", "R2"), ("X5", "R8")]

# Task assignment function
def assign_task_to_robot():
    for robot in robots:
        if tasks:
            start, goal = tasks.pop(0)
            if robot["curr_node"] != start:
                path_to_start = find_path(robot["curr_node"], start)
            else:
                path_to_start = []
            task_path = find_path(start, goal)
            robot["path"] = path_to_start + task_path
            robot["path"]= robot["path"][1:]
            # print(f"Robot {robot['id']} assigned path: {robot['path'].pop(0)}")
            robot["goal"] = goal
            robot["step"] = 0
            robot["status"] = "busy"
            robot["next_node"] = robot["path"][0] if robot["path"] else None

def move_robot(robot, node=None):
    if node is None:
        global task_counter
        # Move the robot to the next node
        robot["curr_node"] = robot["next_node"]
        message = f"R{robot['id']}_{robot['curr_node']}"
        # command_pub.publish(message)
        ProccessCommand(message)
        # print(f"Robot {robot['id']} moving to {robot['next_node']} (step {robot['step'] + 1})")
        robot["step"] += 1
        if robot['step'] < len(robot["path"]):
            robot['next_node'] = robot['path'][robot['step']]
        else:
            robot['next_node'] = None

        # Check if task is completed
        if robot["curr_node"] == robot["path"][-1]:
            # print(f"Task {task_counter} completed by Robot {robot['id']}!")
            task_counter += 1
            robot["status"] = "free"
            robot["path"] = []
            robot["goal"] = None
            robot["next_node"] = None
            # Assign new task immediately
            assign_task_to_robot()
    else:
        # For Deadlock resolving
        # print(f"Robot {robot['id']} Docking to {node} (step {robot['step'] + 1})")
        robot["next_node"] = robot["curr_node"]
        robot["curr_node"] = node
        message = f"R{robot['id']}_{robot['curr_node']}"
        # command_pub.publish(message)
        ProccessCommand(message)
        robot["path"].insert(robot["step"], node)
        robot["step"]+=1
        robot["path"].insert(robot["step"], robot["curr_node"])
        robot["status"] = "wait"

translate = {
    "X1": {"I1": "S"},
    "I1": {"X1": "W", "I2": "S", "R1": "D"},
    "I2": {"I1": "W", "I3": "S", "I4": "D"},
    "I3": {"I2": "W", "R2": "D", "X2": "S"},
    "I4": {"I2": "A", "I5": "D", "R3": "S"},
    "I5": {"I4": "A", "I6": "W", "I7": "S", "I8": "D"},
    "I6": {"I5": "S", "R5": "A", "X3": "W"},
    "I7": {"I5": "W", "R4": "A", "X4": "S"},
    "I8": {"I5": "A", "I9": "D", "R6": "S"},
    "I9": {"I8": "A", "I10": "W", "I11": "S"},
    "I10": {"I9": "S", "R7": "A", "X5": "W"},
    "I11": {"I9": "W", "R8": "A", "X6": "S"},
    "R1": {"I1": "A"},
    "R2": {"I3": "A"},
    "R3": {"I4": "A"},
    "R4": {"I7": "D"},
    "R5": {"I6": "D"},
    "R6": {"I8": "W"},
    "R7": {"I10": "D"},
    "R8": {"I11": "D"},
    "X2": {"I3": "W"},
    "X3": {"I6": "S"},
    "X4": {"I7": "W"},
    "X5": {"I10": "S"},
    "X6": {"I11": "W"}
}

orient = {
    'W': {'W': "W", 'A': "DW", 'S': "S", 'D': "AW"},
    'A': {'W': "AW", 'A': "W", 'S': "DW", 'D': "S"},
    'S': {'W': "S", 'A': "AW", 'S': "W", 'D': "DW"},
    'D': {'W': "AW", 'A': "S", 'S': "AW", 'D': "W"}
}


curr_node = {"R1":"X1", "R2":"X5"}
curr_orientation = {"R1":"S", "R2":"S"}
def ProccessCommand(command): # 'R1_I1'
    node, command = command.split('_')
    # print(node, command, robots[0], robots[1])
    dir_command = translate[curr_node[node]][command]
    curr_node[node] = command
    commands = orient[dir_command][curr_orientation[node]]
    if node == "R2":
        if len(commands)>1:
            for cmd in commands:
                send_to_esp32("192.168.137.117", 80, cmd)
                if cmd == 'A' or cmd == 'D':
                    curr_orientation[node] = dir_command
                time.sleep(1.5)
        else:
            send_to_esp32("192.168.137.117", 80, commands)
            time.sleep(1.5)
    else:
        if len(commands)>1:
            for cmd in commands:
                send_to_esp32("192.168.137.174", 80, cmd)
                if cmd == 'A' or cmd == 'D':
                    curr_orientation[node] = dir_command
                time.sleep(1.5)
        else:
            send_to_esp32("192.168.137.174", 80, commands)
            time.sleep(1.5)

def send_to_esp32(ip, port, message):
    try:
        # Create a socket object
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Connect to the ESP32 server
        client_socket.connect((ip, port))
        print(f"Connected to ESP32 at {ip}:{port}")

        # Send the message
        print(f"Sending: {message}")
        client_socket.sendall((message + "\n").encode())  # Add newline for ESP32
        client_socket.close()
    except Exception as e:
        print(f"Error: {e}")

# Main simulation loop
task_counter = 1
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    assign_task_to_robot()
    draw_graph()
    # Assign tasks to free robots
    for robot in robots:
        if robot["status"] == "free":
            assign_task_to_robot()
        elif robot["status"] == "busy":
            move_robot(robot)
        else: # robot["status"] == "wait"
            robot["status"] = "busy"
        
        pygame.draw.circle(screen, robot["color"], positions[robot["curr_node"]], 10)

    # Check for conflicts
    for robot in robots:
        if robot["status"] == "busy" and robot["next_node"]:
            for other_robot in robots:
                if (other_robot["id"] != robot["id"] and other_robot["status"] == "busy"):
                    if other_robot.get("next_node") == robot["next_node"]:
                        # Conflict detected, set the current robot to wait
                        # print("Conflict 1")
                        if len(robot['path'][robot['step']:]) > len(other_robot['path'][other_robot['step']:]):
                            other_robot["status"] = "wait"
                            break
                        else:
                            robot["status"] = "wait"
                            break
                    if other_robot.get("next_node") == robot["curr_node"] and robot.get("next_node") == other_robot["curr_node"]:
                        # send one of the robots to Docking point: implement later
                        # print("Conflict 2")
                        docking_points1 = set(graph.neighbors(robot["curr_node"])) - set(other_robot["path"])
                        docking_points2 = set(graph.neighbors(other_robot["curr_node"])) - set(robot["path"])
                        if (len(docking_points1) ^ len(docking_points2)) == 1:
                            if len(docking_points1) != 0:
                                move_robot(robot, list(docking_points1)[0])
                            else:
                                move_robot(other_robot, list(docking_points2)[0])
                        else:
                            if len(robot['path'][robot['step']:]) > len(other_robot['path'][other_robot['step']:]):
                                move_robot(other_robot, list(docking_points2)[0])
                            else:
                                move_robot(robot, list(docking_points1)[0])
                        

    time.sleep(2)
    pygame.display.flip()

pygame.quit()
