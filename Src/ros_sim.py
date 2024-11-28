import pygame
import networkx as nx
import time

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
    return nx.shortest_path(graph, start, goal)

# Robots
robots = [
    {"id": 1, "color": RED, "path": [], "position": None, "step": 0, "goal": None, "status": "free", "next_node": None},
    {"id": 2, "color": BLUE, "path": [], "position": None, "step": 0, "goal": None, "status": "free", "next_node": None},
]

# Tasks: Input as start -> goal pairs
tasks = [("X1", "X4"), ("X2", "X6"), ("X4", "X3"), ("X6", "X1"), ("X3", "X6")]

# Task assignment function
def assign_task_to_robot(robot):
    if tasks:
        start, goal = tasks.pop(0)
        robot["path"] = find_path(start, goal)
        robot["goal"] = goal
        robot["step"] = 0
        robot["status"] = "busy"
        robot["next_node"] = robot["path"][0] if robot["path"] else None

# Main simulation loop
task_counter = 1
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    draw_graph()

    # Assign tasks to free robots
    for robot in robots:
        if robot["status"] == "free":
            assign_task_to_robot(robot)

    # Update next_node for all robots
    for robot in robots:
        if robot["status"] == "busy":
            current_step = robot["step"]
            if current_step + 1 < len(robot["path"]):
                robot["next_node"] = robot["path"][current_step + 1]
            else:
                robot["next_node"] = None  # Task complete

    # Check for conflicts
    for robot in robots:
        if robot["status"] == "busy" and robot["next_node"]:
            for other_robot in robots:
                if (
                    other_robot["id"] != robot["id"]
                    and other_robot.get("next_node") == robot["next_node"]
                    and other_robot["status"] == "busy"
                ):
                    # Conflict detected, set the current robot to wait
                    robot["status"] = "wait"
                    break

    # Move robots
    for robot in robots:
        if robot["status"] == "busy" and robot["next_node"]:
            # Move the robot to the next node
            robot["position"] = positions[robot["next_node"]]
            robot["step"] += 1

            # Check if task is completed
            if robot["step"] == len(robot["path"]):
                print(f"Task {task_counter} completed by Robot {robot['id']}!")
                task_counter += 1
                robot["status"] = "free"
                robot["path"] = []
                robot["goal"] = None
                robot["next_node"] = None
                # Assign new task immediately
                assign_task_to_robot(robot)

        elif robot["status"] == "wait":
            # Reset status to busy for the next iteration
            robot["status"] = "busy"

        # Draw robot
        if robot["position"]:
            pygame.draw.circle(screen, robot["color"], robot["position"], 10)

    time.sleep(0.5)
    pygame.display.flip()

pygame.quit()
