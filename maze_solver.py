import pygame
import math
from collections import deque
import heapq # For Dijkstra's and A* priority queue

# --- Constants ---
WIDTH = 800
HEIGHT = 800
ROWS = 50
COLS = 50
NODE_SIZE = WIDTH // COLS # Assuming square grid
GRID_COLOR = (128, 128, 128) # Grey for grid lines

# Colors for nodes
WHITE = (255, 255, 255) # Empty
BLACK = (0, 0, 0)     # Wall
ORANGE = (255, 165, 0)  # Start
TURQUOISE = (64, 224, 208) # End
GREEN = (0, 255, 0)   # Open / Visited
RED = (255, 0, 0)     # Closed / Already Visited
PURPLE = (128, 0, 128) # Path

# --- Node Class ---
class Node:
    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.x = row * NODE_SIZE
        self.y = col * NODE_SIZE
        self.color = WHITE
        self.neighbors = []
        self.previous_node = None # For path reconstruction

        # For Dijkstra's and A*
        self.g_score = float("inf") # Cost from start to this node
        self.h_score = float("inf") # Heuristic cost from this node to end
        self.f_score = float("inf") # g_score + h_score

    def get_pos(self):
        return self.row, self.col

    def is_closed(self):
        return self.color == RED

    def is_open(self):
        return self.color == GREEN

    def is_wall(self):
        return self.color == BLACK

    def is_start(self):
        return self.color == ORANGE

    def is_end(self):
        return self.color == TURQUOISE

    def reset(self):
        self.color = WHITE
        self.previous_node = None
        self.g_score = float("inf")
        self.h_score = float("inf")
        self.f_score = float("inf")

    def make_start(self):
        self.color = ORANGE

    def make_end(self):
        self.color = TURQUOISE

    def make_closed(self):
        self.color = RED

    def make_open(self):
        self.color = GREEN

    def make_wall(self):
        self.color = BLACK

    def make_path(self):
        self.color = PURPLE

    def draw(self, screen):
        pygame.draw.rect(screen, self.color, (self.x, self.y, NODE_SIZE, NODE_SIZE))

    def update_neighbors(self, grid):
        self.neighbors = []
        # Check Up
        if self.row > 0 and not grid[self.row - 1][self.col].is_wall():
            self.neighbors.append(grid[self.row - 1][self.col])
        # Check Down
        if self.row < ROWS - 1 and not grid[self.row + 1][self.col].is_wall():
            self.neighbors.append(grid[self.row + 1][self.col])
        # Check Left
        if self.col > 0 and not grid[self.row][self.col - 1].is_wall():
            self.neighbors.append(grid[self.row][self.col - 1])
        # Check Right
        if self.col < COLS - 1 and not grid[self.row][self.col + 1].is_wall():
            self.neighbors.append(grid[self.row][self.col + 1])

    def __lt__(self, other): # For heapq comparison
        return self.f_score < other.f_score

# --- Grid Functions ---
def make_grid(rows, cols):
    grid = []
    for i in range(rows):
        grid.append([])
        for j in range(cols):
            node = Node(i, j)
            grid[i].append(node)
    return grid

def draw_grid(screen, rows, cols):
    for i in range(rows):
        pygame.draw.line(screen, GRID_COLOR, (0, i * NODE_SIZE), (WIDTH, i * NODE_SIZE))
        for j in range(cols):
            pygame.draw.line(screen, GRID_COLOR, (j * NODE_SIZE, 0), (j * NODE_SIZE, HEIGHT))

def draw(screen, grid, rows, cols):
    screen.fill(WHITE)

    for row in grid:
        for node in row:
            node.draw(screen)

    draw_grid(screen, rows, cols)
    pygame.display.update()

def get_clicked_pos(pos):
    y, x = pos
    row = y // NODE_SIZE
    col = x // NODE_SIZE
    return row, col

# --- Algorithms ---

def reconstruct_path(current, draw_function, start_node, end_node):
    while current.previous_node and current.previous_node != start_node:
        current = current.previous_node
        current.make_path()
        draw_function()
    start_node.make_start() # Ensure start node remains orange
    end_node.make_end()     # Ensure end node remains turquoise

# Manhattan distance heuristic (for A*)
def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)

def bfs(draw_function, grid, start, end):
    queue = deque()
    queue.append(start)
    visited = {start} # Keep track of visited nodes

    start.g_score = 0 # No cost to get to start

    while queue:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        current = queue.popleft()

        if current == end:
            reconstruct_path(current, draw_function, start, end)
            end.make_end()
            return True

        for neighbor in current.neighbors:
            if neighbor not in visited:
                visited.add(neighbor)
                neighbor.previous_node = current
                queue.append(neighbor)
                if neighbor != end:
                    neighbor.make_open() # Mark as being explored
        draw_function()

        if current != start:
            current.make_closed() # Mark as finished exploring

    return False # No path found

def dijkstra(draw_function, grid, start, end):
    # Use a min-priority queue (heap) for Dijkstra's
    open_set = [(0, start)] # (cost, node)
    heapq.heapify(open_set)

    # Initialize g_score for all nodes
    for row in grid:
        for node in row:
            node.g_score = float("inf")
    start.g_score = 0

    # Store (node, previous_node) to reconstruct path
    came_from = {}

    # This set keeps track of nodes that have already been considered
    # and for which the shortest path from start has been finalized.
    closed_set = set()

    while open_set:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        # Get node with the smallest g_score
        current_g_score, current = heapq.heappop(open_set)

        if current in closed_set: # Already processed this node optimally
            continue

        closed_set.add(current)

        if current == end:
            reconstruct_path(current, draw_function, start, end)
            end.make_end()
            return True

        for neighbor in current.neighbors:
            if neighbor in closed_set:
                continue

            # Assuming uniform cost of 1 per step
            temp_g_score = current.g_score + 1

            if temp_g_score < neighbor.g_score:
                neighbor.previous_node = current
                neighbor.g_score = temp_g_score
                heapq.heappush(open_set, (neighbor.g_score, neighbor))
                if neighbor != end:
                    neighbor.make_open() # Mark as being explored

        draw_function()

        if current != start:
            current.make_closed() # Mark as finished exploring

    return False # No path found

def a_star(draw_function, grid, start, end):
    # Use a min-priority queue (heap) for A*
    open_set = [(0, start)] # (f_score, node)
    heapq.heapify(open_set)

    # Initialize g_score and f_score for all nodes
    for row in grid:
        for node in row:
            node.g_score = float("inf")
            node.f_score = float("inf")

    start.g_score = 0
    start.f_score = h(start.get_pos(), end.get_pos())

    # This set keeps track of nodes that are in the open_set
    # It's needed for efficient membership checking in the heap
    open_set_hash = {start}

    while open_set:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        # Get node with the smallest f_score
        current = heapq.heappop(open_set)[1] # [1] because open_set stores (f_score, node)
        open_set_hash.remove(current)

        if current == end:
            reconstruct_path(current, draw_function, start, end)
            end.make_end()
            return True

        for neighbor in current.neighbors:
            # Assuming uniform cost of 1 per step
            temp_g_score = current.g_score + 1

            if temp_g_score < neighbor.g_score: # Found a shorter path to neighbor
                neighbor.previous_node = current
                neighbor.g_score = temp_g_score
                neighbor.f_score = temp_g_score + h(neighbor.get_pos(), end.get_pos())
                if neighbor not in open_set_hash:
                    heapq.heappush(open_set, (neighbor.f_score, neighbor))
                    open_set_hash.add(neighbor)
                    if neighbor != end:
                        neighbor.make_open() # Mark as being explored

        draw_function() # Visualize the step

        if current != start:
            current.make_closed() # Mark as finished exploring

    return False # No path found

# --- Main Game Loop ---
def main(screen):
    grid = make_grid(ROWS, COLS)

    start = None
    end = None

    running = True
    algorithm_started = False

    while running:
        draw(screen, grid, ROWS, COLS)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            # Left mouse button to draw walls, start, end
            if pygame.mouse.get_pressed()[0]:
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos)
                node = grid[row][col]
                if not start and node != end:
                    start = node
                    start.make_start()
                elif not end and node != start:
                    end = node
                    end.make_end()
                elif node != start and node != end:
                    node.make_wall()

            # Right mouse button to erase
            elif pygame.mouse.get_pressed()[2]:
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos)
                node = grid[row][col]
                node.reset()
                if node == start:
                    start = None
                elif node == end:
                    end = None

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_1 and start and end and not algorithm_started:
                    for row_nodes in grid:
                        for node in row_nodes:
                            node.update_neighbors(grid)
                    algorithm_started = True
                    print("Running BFS...")
                    bfs(lambda: draw(screen, grid, ROWS, COLS), grid, start, end)
                    algorithm_started = False
                
                if event.key == pygame.K_2 and start and end and not algorithm_started:
                    for row_nodes in grid:
                        for node in row_nodes:
                            node.update_neighbors(grid)
                    algorithm_started = True
                    print("Running Dijkstra's...")
                    dijkstra(lambda: draw(screen, grid, ROWS, COLS), grid, start, end)
                    algorithm_started = False

                if event.key == pygame.K_3 and start and end and not algorithm_started:
                    for row_nodes in grid:
                        for node in row_nodes:
                            node.update_neighbors(grid)
                    algorithm_started = True
                    print("Running A* Search...")
                    a_star(lambda: draw(screen, grid, ROWS, COLS), grid, start, end)
                    algorithm_started = False

                if event.key == pygame.K_c: # Clear board
                    start = None
                    end = None
                    grid = make_grid(ROWS, COLS)
                    algorithm_started = False
                    print("Board cleared!")

                if event.key == pygame.K_q: # Quit
                    running = False


    pygame.quit()

# --- Initialize Pygame ---
if __name__ == "__main__":
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Maze Solver Visualizer")
    main(screen)