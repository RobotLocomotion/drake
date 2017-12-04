This directory contains various urdf models describing the dual iiwa arm setup
for the box rotation simulation. The inter-arm distance defined in these urdf's
is particularly important in making sure the open-loop simulation actually works.
Changing this value could cause the open-loop simulation to break. Future versions
will probablly just check and/or enforce it.

There are two collision models defined for the arms:
1. Overlapping spheres, representing relevant parts of the arm (i.e., parts that are
most likely to make contact with the box).
2. Cylinders, representing the relevant parts of the arm.

Since Drake does not yet have the ability to choose what elements to visualize (i.e,
visual geometry vs collision geometry), each collision model has two versions:
1. One which only visualizes the collision geometry (e.g., `dual_iiwa14_primitive_sphere_collision_only.urdf`).
2. Another which only visualizes visual geometries (but still includes collision geometries), (e.g., 
`dual_iiwa14_primitive_sphere_visual_collision.urdf`).

Finally `dual_iiwa14_visual_only.urdf` only contains visual geometry (no collision geometry
is defined in this urdf file).

The `box.urdf` model is a (very rough) approximate model of the Home Depot extra large cardboard box.
