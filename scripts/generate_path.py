#!/usr/bin/env python3

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from yaml import load, dump, FullLoader
import skimage
from os.path import join, isdir
import numpy as np
from skimage.morphology import skeletonize
import argparse

parser = argparse.ArgumentParser(description='Generate pickle file (.pkl) from trajectory image (.pgm)')
parser.add_argument('dir_path', metavar='dir_path', help='path to directory containing maps and trajectories')
parser.add_argument('map_name', metavar='map_name', help='name of the map file without .yaml extension')
parser.add_argument('--display', action='store_false', help='flag to display images illustrating the process of generating path')

args = parser.parse_args()

if not isdir(args.dir_path):
    print('The path specified does not exist')
    sys.exit()

MAPS_dir = args.dir_path
MAP_name = args.map_name
TRAJECTORY_file = MAP_name + '_trajectory.pgm'


trajectory_file = join(MAPS_dir, TRAJECTORY_file)
with open(join(MAPS_dir, MAP_name+'.yaml'), 'r') as f:
    map_meta = load(f, Loader=FullLoader)

print(map_meta)

map_img = mpimg.imread(join(MAPS_dir, map_meta['image']))
trajectory_img = mpimg.imread(join(MAPS_dir,TRAJECTORY_file))

if(args.display==True):
    plt.figure(figsize=(15, 15))
    plt.imshow(map_img, cmap='gray')
    plt.title('original map')
    plt.show()


occupied = np.zeros_like(trajectory_img)
free     = np.zeros_like(trajectory_img)
path = np.zeros_like(trajectory_img)

occupied[trajectory_img > map_meta['occupied_thresh']] = 1.0
free[trajectory_img < map_meta['free_thresh']] = 1.0

B = np.argwhere(occupied | free)
(ystart, xstart), (ystop, xstop) = B.min(0), B.max(0) + 1

path = free[ystart:ystop, xstart:xstop]

skeleton = skeletonize(path)

if(args.display==True):
    plt.figure(figsize=(15, 15))
    plt.imshow(skeleton, cmap='gray')
    plt.title('path skeletonized', fontsize=40)
    plt.show()

map_combined = np.array(map_img)
map_combined[skeleton==1] = 0


if(args.display==True):
    plt.figure(figsize=(15, 15))
    plt.imshow(map_combined, cmap='gray')
    plt.title('original map and trajectory', fontsize=40)
    plt.show()

from itertools import permutations
import sys
sys.setrecursionlimit(999999999)

OFFSETS = [-1, 0, 1]
NEIGHBORHOOD = [(dy, dx) for dx in OFFSETS for dy in OFFSETS]

available = np.copy(skeleton).astype(np.bool)

def dfs(point, target, available, path, remaining_steps=15):
    x = point[0]
    y = point[1]
    
    if (remaining_steps == 0) and (point == target).all(): return True
    
    # Lower & upper bounds
    if x < 0 or x >= available.shape[1]: return False
    if y < 0 or y >= available.shape[0]: return False
    
    if not available[y, x]: return False
    
    remaining_steps = max(0, remaining_steps-1)
    
    available[y, x] = False
    path.append(point)
    
    for dx, dy in NEIGHBORHOOD:
        child_x = x + dx
        child_y = y + dy
        
        if dfs((child_x, child_y), target, available, path, remaining_steps):
            return True

    path.pop()
    return False

starting_points = np.transpose(np.nonzero(available))
for starting_point in starting_points:
    path = []
    
    if dfs(starting_point[::-1], starting_point[::-1], available, path):
        break

path = path[::-8]        
    
path_img = np.zeros_like(skeleton)
#path_img = skeleton

for point in path:
    path_img[point[1], point[0]] = 1


if(args.display==True):
    plt.figure(figsize=(15, 15))
    plt.imshow(path_img, cmap='gray')
    plt.title('path extracted', fontsize=40)
    plt.show()

print(len(path))

import pickle

data = {}

res = map_meta['resolution']

data['resolution'] = res
data['origin'] = [
    map_meta['origin'][0] + xstart * res,
    map_meta['origin'][1] + (map_img.shape[0] - ystop - 1) * res
]
data['pathpoints'] = [
    (data['origin'][0] + x*res, data['origin'][1] + (path_img.shape[0] - y - 1)*res)
    for x, y in path
]

with open(join(MAPS_dir, MAP_name+'.pkl'), 'wb') as f:
    pickle.dump(data, f, 2)

print('pickle file saved to '+ MAPS_dir)