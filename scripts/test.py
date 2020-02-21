#!/usr/bin/env python

import argparse

parser = argparse.ArgumentParser(description='Generate pickle file (.pkl) from trajectory image (.pgm)')
parser.add_argument('dir_path', metavar='dir_path', help='path to directory containing maps and trajectories')
parser.add_argument('map_name', metavar='map_name', help='name of the map file without .yaml extension')

args = parser.parse_args()


print args