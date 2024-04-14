import helper
from PIL import Image
import numpy as np

if __name__ == "__main__":
    grid = helper.read_pgm()

    grid_expanded_obstacles = helper.take_local(grid, 3, np.min)

    Image.fromarray(grid_expanded_obstacles).save("vis1.png")

    start = (40, 33)
    end = (76, 130)

    result_path = helper.find_route_astar(grid_expanded_obstacles, start, end)
    filtered_path = helper.filter_path(result_path)

    if result_path is not None:
        print("Grid with the shortest path marked as gray line:")
        img = Image.fromarray(helper.visualize(grid.copy(), result_path, 100))
        img.save("vis2.png")
    else:
        print("No valid path exists.")

    if filtered_path is not None:
        img = Image.fromarray(helper.visualize(grid.copy(), filtered_path, 100))
        img.save("vis3.png")
    else:
        print("No valid path exists.")
