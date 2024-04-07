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

file = open("maps/yahboomcar.pgm", "rb")
raster = read_pgm(file)
for i in raster:
    for j in i:
        if j == 0:
            print(" ", end = "")
        elif j == 254:
            print("*", end = "")
        else:
            print("-", end = "")
    print("\n", end = "")