#!/usr/bin/env python3

import pygame
import networkx as nx
import time
import heapq
import rospy
from std_msgs.msg import String

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
tasks = [("X1", "X6")]

# Task assignment function
def assign_task_to_robot(robot):
    if tasks:
        start, goal = tasks.pop(0)
        if robot["curr_node"] != start:
            path_to_start = find_path(robot["curr_node"], start)
        else:
            path_to_start = []
        task_path = find_path(start, goal)
        robot["path"] = path_to_start + task_path
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
        command_pub.publish(message)
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
            assign_task_to_robot(robot)
    else:
        # For Deadlock resolving
        # print(f"Robot {robot['id']} Docking to {node} (step {robot['step'] + 1})")
        robot["next_node"] = robot["curr_node"]
        robot["curr_node"] = node
        message = f"R{robot['id']}_{robot['curr_node']}"
        command_pub.publish(message)
        robot["path"].insert(robot["step"], node)
        robot["step"]+=1
        robot["path"].insert(robot["step"], robot["curr_node"])
        robot["status"] = "wait"

# Main simulation loop
task_counter = 1
running = True
rospy.init_node('LogicSystems', anonymous=True)
command_pub = rospy.Publisher('/Logic_to_CCS', String, queue_size=10)
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    draw_graph()
    robot = robots[0]
    # print(f"Robot {robot['id']} State: step={robot['step']}, path={robot['path']}, status={robot['status']}, curr_node={robot['curr_node']}, next_node={robot['next_node']}")
    robot = robots[1]
    # print(f"Robot {robot['id']} State: step={robot['step']}, path={robot['path']}, status={robot['status']}, curr_node={robot['curr_node']}, next_node={robot['next_node']}")
    # Assign tasks to free robots
    for robot in robots:
        if robot["status"] == "free":
            assign_task_to_robot(robot)
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
    robot = robots[0]
    # print(f"Robot {robot['id']} State: step={robot['step']}, path={robot['path']}, status={robot['status']}, curr_node={robot['curr_node']}, next_node={robot['next_node']}")
    robot = robots[1]
    # print(f"Robot {robot['id']} State: step={robot['step']}, path={robot['path']}, status={robot['status']}, curr_node={robot['curr_node']}, next_node={robot['next_node']}")

    # input()

pygame.quit()
