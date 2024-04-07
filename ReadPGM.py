from PIL import Image
import numpy as np


# Open the PGM file
image = Image.open("1.pgm")
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