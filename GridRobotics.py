import pygame
import random
import time
import heapq

GRID_SIZE = 10
CELL_SIZE = 50
WINDOW_SIZE = GRID_SIZE * CELL_SIZE
WINDOW_HEIGHT = WINDOW_SIZE
OBSTACLE_COUNT = 30
MOVE_INTERVAL = 3  
STEP_DELAY = 0.5   

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
BLUE = (0, 0, 255)
LIGHT_BLUE = (173, 216, 230)  

pygame.init()
window = pygame.display.set_mode((WINDOW_SIZE, WINDOW_HEIGHT))
pygame.display.set_caption("Dynamic Robot Navigation")

def initialize_grid(start, end):
    grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    obstacles = []
    while len(obstacles) < OBSTACLE_COUNT:
        x, y = random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1)
        if grid[x][y] == 0 and (x, y) != start and (x, y) != end:  
            grid[x][y] = 1
            obstacles.append((x, y))
    return grid, obstacles

def draw_grid(grid, robot, end, path, obstacles, start=None, temp_highlight=None):
    window.fill(WHITE)

    for x in range(GRID_SIZE):
        for y in range(GRID_SIZE):
            color = WHITE if grid[x][y] == 0 else RED
            pygame.draw.rect(window, color, (y * CELL_SIZE, x * CELL_SIZE, CELL_SIZE, CELL_SIZE))
            pygame.draw.rect(window, BLACK, (y * CELL_SIZE, x * CELL_SIZE, CELL_SIZE, CELL_SIZE), 1)

    if temp_highlight:
        pygame.draw.rect(window, LIGHT_BLUE, 
                         (temp_highlight[1] * CELL_SIZE, temp_highlight[0] * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    if start:
        pygame.draw.circle(window, GREEN, (start[1]*CELL_SIZE+CELL_SIZE//2, start[0]*CELL_SIZE+CELL_SIZE//2), CELL_SIZE//4)
    if end:
        pygame.draw.circle(window, BLUE, (end[1]*CELL_SIZE+CELL_SIZE//2, end[0]*CELL_SIZE+CELL_SIZE//2), CELL_SIZE//4)

    for x, y in path:
        pygame.draw.rect(window, YELLOW, (y * CELL_SIZE, x * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    pygame.display.update()

def a_star(grid, start, end):
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, end)}

    while open_list:
        _, current = heapq.heappop(open_list)
        if current == end:
            return reconstruct_path(came_from, current)
        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < GRID_SIZE and 0 <= neighbor[1] < GRID_SIZE and grid[neighbor[0]][neighbor[1]] == 0:
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, end)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))
    return []

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def reconstruct_path(came_from, current):
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path

def main():
    start, end = None, None
    robot = None
    grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    path, obstacles = [], []
    last_move_time = time.time()
    setting_start = True
    temp_highlight = None

    draw_grid(grid, robot, end, path, obstacles)  

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEMOTION:
                x, y = event.pos[1] // CELL_SIZE, event.pos[0] // CELL_SIZE
                if 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE and grid[x][y] == 0:
                    temp_highlight = (x, y)
                else:
                    temp_highlight = None
            elif event.type == pygame.MOUSEBUTTONDOWN:
                x, y = event.pos[1] // CELL_SIZE, event.pos[0] // CELL_SIZE
                if setting_start:
                    start = (x, y)
                    setting_start = False
                elif not setting_start and end is None:
                    end = (x, y)
                    grid, obstacles = initialize_grid(start, end)
                    robot = start
                    path = a_star(grid, start, end)

        if start and end and time.time() - last_move_time > MOVE_INTERVAL:
            for i in range(len(obstacles)):
                x, y = obstacles[i]
                grid[x][y] = 0
                while True:
                    nx, ny = x + random.choice([-1, 0, 1]), y + random.choice([-1, 0, 1])
                    if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and grid[nx][ny] == 0 and (nx, ny) != start and (nx, ny) != end:
                        obstacles[i] = (nx, ny)
                        grid[nx][ny] = 1
                        break
            last_move_time = time.time()
            path = a_star(grid, robot, end)  # Recalculate the path dynamically

        if path and robot != end:
            next_step = path.pop(0)
            robot = next_step
            if robot == end:
                print("Reached destination!")
                running = False

        draw_grid(grid, robot, end, path, obstacles, start=start, temp_highlight=temp_highlight)
        time.sleep(STEP_DELAY)

main()
pygame.quit()
