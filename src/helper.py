import numpy as np
from heapq import heappush, heappop
from ReadPGM import read_pgm
from PIL import Image

def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def find_route_astar(grid, start, end):
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
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            
            # Set the cells along the path to -1
            for cell in path:
                grid[cell[0], cell[1]] = 100
            
            return grid
        
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


# Example usage
grid = read_pgm()
start = (40, 33)
end = (76, 130)

result = find_route_astar(grid, start, end)

if result is not None:
    print("Grid with the shortest path marked as gray line:")
    img = Image.fromarray(result)
    img.show()
else:
    print("No valid path exists.")