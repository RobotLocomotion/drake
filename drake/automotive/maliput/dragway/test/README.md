# How to Run `dragway_to_urdf`

To run `dragway_to_urdf`, execute:

    $ cd [directory for holding the generated URDF and OBJ files]
    $ bazel run //drake/automotive/maliput/dragway:dragway_to_urdf -- \
          --dirpath=[dirpath] \
          --length=[length of road in meters] \
          --num_lanes=[number of lanes] \
          --shoulder_width=[width of shoulder in meters]
