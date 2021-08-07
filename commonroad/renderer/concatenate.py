from os import path
import os
from PIL import Image

TILE_SIZE = 2048

def concate_tiles(target_dir, filename, image_names, width_num, height_num): 
    dst = Image.new('RGB', (width_num * TILE_SIZE, height_num * TILE_SIZE))
    image_number = len(image_names)

    for i in range(image_number):
        full_path = path.join(os.getcwd(), target_dir, filename, "materials", "textures", image_names[i])
        img = Image.open(full_path)

        x = int(i / height_num) * TILE_SIZE
        y = (height_num - (i % height_num) - 1) * TILE_SIZE       
        dst.paste(img, (x, y))
    
    return dst
