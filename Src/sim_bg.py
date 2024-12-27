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
    {"id": 1, "color": RED, "path": [], "step": 0, "goal": None, "status": "free", "curr_node": "X1"},
    {"id": 2, "color": BLUE, "path": [], "step": 0, "goal": None, "status": "free", "curr_node": "X5"},
]

# Tasks: Input as start -> goal pairs
# tasks = [("X1", "X4"), ("X2", "X6"), ("X4", "X3"), ("X6", "X1"), ("X3", "X6")] # Set 1 of tasks success
tasks = [("X1", "X6"), ("X5", "X2")]
# tasks = [("X1", "X4"), ("X5", "X2")] # Set 2 of tasks
# tasks = [("X1", "X2"), ("X5", "X3"), ("X2", "X6"), ("X3", "X1")] # Set 3

# Task assignment function
def assign_task_to_robot():
    for robot in robots:
        if tasks and robot['status']=='free':
            start, goal = tasks.pop(0)
            if robot["curr_node"] != start:
                path_to_start = find_path(robot["curr_node"], start)
            else:
                path_to_start = []
            task_path = find_path(start, goal)
            robot["path"] = path_to_start + task_path
            robot["path"]= robot["path"][1:]
            print(f"Robot {robot['id']} assigned path: {robot['path']}")
            robot["goal"] = goal
            robot["step"] = 0
            robot["status"] = "busy"

def move_robot():
    global task_counter
    message = []
    for robot in robots:
        if robot['status'] in ['busy', 'conflict']:
            robot["curr_node"] = robot['path'].pop(0)
            message.append(robot["curr_node"])
            robot["step"] += 1
            if robot["curr_node"] == robot["goal"]:
                task_counter += 1
                robot["status"] = "free"
                robot["path"] = []
                robot["goal"] = None
        else:
            message.append('N')
    print(f"Moves: {message}")
    ProccessCommand(message)
        

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
    'W': {'W': "W", 'A': "D", 'S': "S", 'D': "A"},
    'A': {'W': "A", 'A': "W", 'S': "D", 'D': "S"},
    'S': {'W': "S", 'A': "A", 'S': "W", 'D': "D"},
    'D': {'W': "A", 'A': "S", 'S': "A", 'D': "W"}
}


curr_node = {"R1":"X1", "R2":"X5"}
curr_orientation = {"R1":"S", "R2":"S"}
dir_speed1 = {"W":1300, "S":1300, "A":750, "D":750, "N":0}
dir_speed2 = {"W":1300, "S":1300, "A":750, "D":750, "N":0}
def ProccessCommand(command): # 'R1_I1'
    # print(node, command, robots[0], robots[1])

    if command[0] != 'N':
        dir_command1 = translate[curr_node["R1"]][command[0]]
        curr_node["R1"] = command[0]
        commands1 = orient[dir_command1][curr_orientation["R1"]]
        if commands1 == 'A' or commands1 == 'D':
            curr_orientation["R1"] = dir_command1
    else:
        commands1 = "N"
    if command[1] != 'N':
        dir_command2 = translate[curr_node["R2"]][command[1]]
        curr_node["R2"] = command[1]
        commands2 = orient[dir_command2][curr_orientation["R2"]]
        if commands2 == 'A' or commands2 == 'D':
            curr_orientation["R2"] = dir_command2
    else:
        commands2="N"
    # broadcast_to_network(80, (commands1, dir_speed1[commands1], commands2, dir_speed2[commands2]))

def broadcast_to_network(port, message):
    try:
        broadcast_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        broadcast_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)    
        broadcast_address = ("<broadcast>", port)
        # print(f"Broadcasting to {broadcast_address} - Message: {message}")
        broadcast_socket.sendto((message + "\n").encode(), broadcast_address)
        broadcast_socket.close()
        time.sleep(1.5)
    except Exception as e:
        print(f"Error: {e}")

# Main simulation loop
ttc = len(tasks)
task_counter = 0
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    if ttc == task_counter:
        running=False
    for robot in robots:
        print(f"{robot}")
    assign_task_to_robot()
    draw_graph()
    move_robot()
    for robot in robots:
        if robot['status'] in ['wait', 'conflict']:
            robot['status'] = 'busy'
        pygame.draw.circle(screen, robot["color"], positions[robot["curr_node"]], 10)
    for robot in robots:
        print(f"{robot}")
    # Check for conflicts
    for robot in robots[::-1]:
        if robot["status"] == "busy":
            for other_robot in robots[::-1]:
                if (other_robot["id"] != robot["id"] and other_robot["status"] == "busy"):
                    if other_robot['path'][0] == robot["path"][0]:
                        # Conflict detected, set the current robot to wait
                        print("Conflict 1")
                        if len(robot['path']) > len(other_robot['path']):
                            other_robot["status"] = "wait"
                            robot['status'] = 'conflict'
                            break
                        else:
                            robot["status"] = "wait"
                            other_robot['status'] = 'conflict'
                            break
                    elif other_robot["path"][0] == robot["curr_node"] and robot["path"][0] == other_robot["curr_node"]:
                        # send one of the robots to Docking point: implement later
                        print("Conflict 2")
                        docking_points1 = (set(graph.neighbors(robot["curr_node"])) - set(other_robot["path"])) - set([other_robot["curr_node"]])
                        docking_points2 = (set(graph.neighbors(other_robot["curr_node"])) - set(robot["path"])) - set([robot["curr_node"]])
                        # print(docking_points1, docking_points2)
                        if (len(docking_points1) ^ len(docking_points2)) == 1:
                            if len(docking_points1) != 0:
                                nxt_node = list(docking_points1)[0]
                                robot['path'] = [nxt_node, robot['curr_node']] + robot["path"]
                                robot['status'] = 'conflict'
                                other_robot['status'] = 'conflict'
                                break
                            else:
                                nxt_node = list(docking_points2)[0]
                                other_robot['path'] = [nxt_node, other_robot['curr_node']] + other_robot["path"]
                                robot['status'] = 'conflict'
                                other_robot['status'] = 'conflict'
                                break
                        else:
                            if len(robot['path']) > len(other_robot['path']):
                                nxt_node = list(docking_points2)[0]
                                other_robot['path'] = [nxt_node, other_robot['curr_node']] + other_robot["path"]
                                robot['status'] = 'conflict'
                                other_robot['status'] = 'conflict'
                                break
                            else:
                                nxt_node = list(docking_points1)[0]
                                robot['path'] = [nxt_node, robot["curr_node"]] + robot["path"]
                                robot['status'] = 'conflict'
                                other_robot['status'] = 'conflict'
                                break
    for robot in robots:
        print(f"{robot}")

    time.sleep(2)
    # input()
    pygame.display.flip()

pygame.quit()
