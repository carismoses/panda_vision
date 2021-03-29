""" Massachusetts Institute of Technology

Izzybrand, 2020
"""
import argparse
import cv2
import numpy as np
import pickle
from PIL import Image, ImageDraw, ImageFont
from rotation_util import *

# Load the predefined dictionary
letters = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', \
            'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z']
letter_map = {block_id:letter for block_id, letter in enumerate(letters)}

ppcm = 300 # pixels per cm specifies the resolution of the saved image
marker_scale = 0.9 # scale of the aruco tag on the face
dpi = ppcm*2.54

# correspond to faces ['+X', '-X', '+Y', '-Y', '+Z', '-Z']
colors = ['red', 'cyan', 'green', 'yellow', 'blue', 'magenta']

def generate_texture(block_id, block_dimensions):
    letter = letter_map[block_id]
    d_x, d_y, d_z = block_dimensions
    face_dimensions_list = np.array([[d_y, d_z],  # +x
                                     [d_y, d_z],  # -x
                                     [d_x, d_z],  # +y
                                     [d_x, d_z],  # -y
                                     [d_x, d_y],  # +z
                                     [d_x, d_y]]) # -z

    for i in range(6):
        # get the dimensions of the face
        face_dimensions_cm = face_dimensions_list[i] * 100
        
        # the color corresponding to this face
        color = colors[i]

        # and insert the letter marker into an image for the face
        marker_size_cm = face_dimensions_cm.min() * marker_scale
        marker_size = int(marker_size_cm * ppcm)
        face_dimensions_px = (face_dimensions_cm*ppcm).astype(int)
        face_image = Image.new('RGB', tuple(face_dimensions_px), color=color)
        d = ImageDraw.Draw(face_image)
        
        fontsize = 1
        font = ImageFont.truetype('/Library/Fonts/Arial.ttf', fontsize)
        while all([size < marker_size for size in d.textsize(letter, font=font)]):
            fontsize += 1
            font = ImageFont.truetype('/Library/Fonts/Arial.ttf', fontsize)
        fontsize -= 1
        font = ImageFont.truetype('/Library/Fonts/Arial.ttf', fontsize)
        top_left = (face_dimensions_px-np.array(d.textsize(letter, font=font)))/2
        top_left -= [0, 100] # for some reason PIL adds blank space to top of letter
        d.text(top_left, letter, font=font, fill='white', stroke_width=20, stroke_fill='black')

        # add a 2-pixel border to the image to show where to cut with scissors
        d.rectangle([0, 0, face_dimensions_px[0], face_dimensions_px[1]],
                    outline=(0,0,0), width=2)

        # save the image at the correct size (PPCM) to print
        face_filename = f'../color_tags/block_{block_id}_tag_{color}.png'
        face_image.save(face_filename, format='PNG', dpi=(dpi, dpi))
        
        print(f'Saved image to {face_filename}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Create aruco tags for a block.')
    parser.add_argument('block_id', type=int, help='The id number of this block')
    parser.add_argument('--dimensions', nargs=3, metavar=('dx', 'dy', 'dz'), type=float,
                    help='Dimensions of the block in meters', default=None)
    args = parser.parse_args()

    generate_texture(args.block_id, args.dimensions)
