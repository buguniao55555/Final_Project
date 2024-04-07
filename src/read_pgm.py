from PIL import Image
import numpy as np

def read_pgm(pgmf):
    """Return a raster of integers from a PGM as a list of lists."""
    pgmf.readline()
    (width, height) = [int(i) for i in pgmf.readline().split()]
    depth = int(pgmf.readline())
    assert depth <= 255

    raster = []
    for y in range(height):
        row = []
        for y in range(width):
            data = ord(pgmf.read(1))
            row.append(data)
        raster.append(row)
    return raster

def display_in_txt(raster):
    for i in raster:
        for j in i:
            if j == 0:
                print(" ", end = "")
            elif j == 254:
                print("*", end = "")
            else:
                print("-", end = "")
        print("\n", end = "")

def crop_data(raster):
    top, bottom, left, right = None
    for i in range(len(raster)):
        for j in range(len(raster[0])):
            if raster[i][j] 

file = open("maps/yahboomcar.pgm", "rb")
raster = read_pgm(file)

