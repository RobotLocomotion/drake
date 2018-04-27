# Description

Dragway is an implementation of Maliput's API that allows users to instantiate
a multilane dragway.  All lanes in the dragway are straight, parallel, and in
the same segment. The ends of each lane are connected together via a "magical
loop" that results in vehicles traveling on the Dragway's lanes instantaneously
teleporting from one end of the lane to the opposite end of the lane.

The number of lanes and their lengths, widths, and shoulder widths are all user
specifiable.

# How to Run `dragway_to_urdf`

The executable `dragway_to_urdf` allows one create a URDF representation of a
dragway. To run `dragway_to_urdf`, execute:

    bazel run //automotive/maliput/dragway:dragway_to_urdf -- \
          --dirpath=[dirpath] \
          --file_name_root=[file name root] \
          --lane_width=[lane width] \
          --length=[length of road in meters] \
          --num_lanes=[number of lanes] \
          --shoulder_width=[width of shoulder in meters]

For an explanation on what the above-mentioned parameters mean, execute:

    bazel run //automotive/maliput/dragway:dragway_to_urdf -- --help

One the above command is executed, the following files should exist:

  1. `[dirpath]/[file name root].urdf` -- a [URDF](http://wiki.ros.org/urdf)
     description of the dragway.
  2. `[dirpath]/[file name root].obj` -- a
     [Wavefront OBJ](https://en.wikipedia.org/wiki/Wavefront_.obj_file) mesh of
     the dragway.
  3. `[dirpath]/[file name root].mtl` -- a material file that applies textures
     and colors to the above Wavefront OBJ file.
