#!/usr/bin/env python

# Computes the spatial inertia of a solid cuboid.
# See: https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors

import argparse
import collections

def ComputeSpatialInertia(x, y, z, m):
  spatial_inertia = collections.namedtuple('SpatialInertia',
    ['ixx', 'iyy', 'izz'])

  # Assmes X = depth, Y = width, Z = height

  ixx = m / 12.0 * (y ** 2 + z ** 2)
  iyy = m / 12.0 * (x ** 2 + z ** 2)
  izz = m / 12.0 * (x ** 2 + y ** 2)

  return spatial_inertia(ixx, iyy, izz);

if __name__ == '__main__':
  parser = argparse.ArgumentParser(
      description='Computes the spatial inertia of a solid cuboid.')

  parser.add_argument('x_length', metavar='x', type=float,
                      help='the cuboid\'s length along its x-axis')
  parser.add_argument('y_length', metavar='y', type=float,
                      help='the cuboid\'s length along its y-axis')
  parser.add_argument('z_length', metavar='z', type=float,
                      help='the cuboid\'s length along its z-axis')
  parser.add_argument('mass', metavar='m', type=float,
                      help='the mass of the cuboid')

  args = parser.parse_args()

  x = args.x_length
  y = args.y_length
  z = args.z_length
  m = args.mass

  print "Computing spatial inertia of solid cuboid with properties:\n" \
        "  - x axis length = {0}\n" \
        "  - y axis length = {1}\n" \
        "  - z axis length = {2}\n" \
        "  - mass = {3}".format(x, y, z, m)

  spatial_inertia = ComputeSpatialInertia(x, y, z, m)

  print "Results\n"\
        "  - ixx = {0}\n" \
        "  - iyy = {1}\n" \
        "  - izz = {2}".format(spatial_inertia.ixx, spatial_inertia.iyy,
                               spatial_inertia.izz)
