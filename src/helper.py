import numpy as np
from heapq import heappush, heappop
from PIL import Image
import numpy as np

def read_pgm():
    # Open the PGM file
    image = Image.open("maps/yahboomcar.pgm")
    image = image.convert("L")
    width, height = image.size
    pixels = image.load()
    left = width
    right = 0
    top = height
    bottom = 0

    # Iterate over the pixels and process them
    for y in range(height):
        for x in range(width):
            pixel_value = pixels[x, y]
            if pixel_value != 205:
                top = min(top, y)
                bottom = max(bottom, y)
                left = min(left, x)
                right = max(right, x)

    # Crop the image to dicard most of the gray area
    image = image.crop((left, top, right+1, bottom+1))

    # save the image to 2D np array for A* algo
    pixel_array = np.array(image)
    
    return pixel_array


def take_local(grid, radius, op):
    r = radius - 1
    paddedd = np.pad(grid, [(r, r), (r, r)], mode="edge")
    result = np.empty_like(grid)
    
    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            result[i, j] = op(paddedd[i: i + 2 * r + 1, j: j + 2 * r + 1])
    return result

def l2norm(a, b):
    return np.hypot(b[0] - a[0], b[1] - a[1])

def construct_path(start, end, came_from):
    path = []
    p = end
    while p != start:
        path.append(p)
        p = came_from[p]
    path.reverse()
    return path

def find_route_astar(grid, start, end, heuristic = l2norm):
    rows, cols = grid.shape
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # Right, Left, Down, Up
    
    open_list = []
    heappush(open_list, (0, start))
    g_score = {start: 0}
    f_score = {start: heuristic(start, end)}
    came_from = {}
    
    while open_list:
        current = heappop(open_list)[1]
        
        if current == end:
            return construct_path(start, end, came_from)
        
        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            
            if (0 <= neighbor[0] < rows and
                0 <= neighbor[1] < cols and
                grid[neighbor[0], neighbor[1]] > 205):
                
                tentative_g_score = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, end)
                    heappush(open_list, (f_score[neighbor], neighbor))
    
    return None

def visualize(grid, path, line_val):
    for cell in path:
        grid[cell[0], cell[1]] = line_val
    return grid
