from ReadPGM import read_pgm
import helper
from PIL import Image
import numpy as np

if __name__ == "__main__":
    grid = read_pgm()

    grid_expanded_obstacles = helper.take_local(grid, 3, np.min)

    Image.fromarray(grid_expanded_obstacles).save("vis1.png")

    start = (40, 33)
    end = (76, 130)

    result_path = helper.find_route_astar(grid_expanded_obstacles, start, end)

    if result_path is not None:
        print("Grid with the shortest path marked as gray line:")
        img = Image.fromarray(helper.visualize(grid, result_path, 100))
        img.save("vis2.png")
    else:
        print("No valid path exists.")
