#!/usr/bin/env python

# Computes the spatial inertia of a solid cuboid.
# See: https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors

import argparse

parser = argparse.ArgumentParser(
  description='Computes the spatial inertia of a solid cuboid.')

parser.add_argument('width', metavar='w', type=float,
                    help='the width of the cuboid')
parser.add_argument('depth', metavar='d', type=float,
                    help='the depth of the cuboid')
parser.add_argument('height', metavar='h', type=float,
                    help='the height of the cuboid')
parser.add_argument('mass', metavar='m', type=float,
                    help='the mass of the cuboid')

args = parser.parse_args()

w = args.width
d = args.depth
h = args.height
m = args.mass

ixx = m / 12.0 * (h ** 2 + d ** 2)
iyy = m / 12.0 * (w ** 2 + d ** 2)
izz = m / 12.0 * (w ** 2 + h ** 2)

print "ixx = {0}".format(ixx)
print "iyy = {0}".format(iyy)
print "izz = {0}".format(izz)
