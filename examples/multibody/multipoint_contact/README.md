# Multi-point contact between convex shapes

This example shows what the `"mujoco_multipoint"` point contact algorithm
changes, by dropping two boxes onto a slab side by side: the box on the left
touches the slab through Drake's default single contact point, the box on the
right through a four-point contact manifold.

## Background

Drake's point contact model resolves each colliding geometry pair to a single
contact point, the point of deepest penetration, even when two flat faces
overlap. A box resting on a table is then supported at one point and cannot
be in static equilibrium there: the support point wanders across the bottom
face and the box rocks. The usual workaround is to decorate a body with
several small spheres so that it touches through several points (see
`examples/multibody/cylinder_with_multicontact`).

The proximity property `("material", "point_contact_algorithm")` selects an
alternative narrowphase for point contact. Its value `"mujoco_multipoint"`
uses MuJoCo's native convex collision detection (GJK/EPA followed by clipping
the two touching faces against each other) to report a contact manifold of up
to four points, each with its own penetration depth, for pairs of `Convex` or
`Mesh` shapes. Face-face and edge-face contacts gain the extra points;
vertex-face contacts still report a single point. Two rules matter for setting
up a scene:

- *Both* geometries of a pair must select `"mujoco_multipoint"`; a pair with
  one geometry left at the default `"single_point"` reports one point.
- Only `Convex` and `Mesh` shapes are eligible. A `Box` shape is not, so this
  example models its boxes as `Convex` shapes built from an in-memory OBJ file
  listing the eight corners (`make_convex_box()` in `falling_box.py`).

The property can be set per geometry, as this example does, or for a whole
scene at once through
`SceneGraphConfig.default_proximity_properties.point_contact_algorithm`.

## The scene

- A 1 m x 0.5 m x 0.05 m slab, a box modeled as a `Convex` shape, welded to
  the world with its top face at z = 0. It selects `"mujoco_multipoint"`.
- Two identical 20 cm x 20 cm x 4 cm boxes of 0.5 kg, also `Convex` shapes.
  The orange box on the left selects `"single_point"`; the blue box on the
  right selects `"mujoco_multipoint"`.
- Both boxes start level, 5 cm above the slab, and fall onto it.

The plant uses `contact_model = "point"` with a 1 ms discrete time step.

## Running

```
bazel run //examples/multibody/multipoint_contact:falling_box
```

Open the Meshcat URL printed at startup. The contact forces are drawn as
arrows, one per contact point, so the left box shows a single arrow and the
right box shows four once it has landed. The console prints, every half
second, the number of contact points each box has with the slab, the height
of the box center above the slab in millimeters (20.00 for a box at rest on
the slab), and the box's angular speed in rad/s:

```
               single_point box               multipoint box
   time   contacts   height    |w|     contacts   height    |w|
   0.50          1    17.46   0.242          4    20.00   0.000
   1.00          1    16.31   0.237          4    20.00   0.000
   ...
   5.00          1    14.50   0.228          4    20.00   0.000
```

Use `--simulation_time`, `--target_realtime_rate`, `--time_step`, and
`--drop_height` to change the run.

## What to look for

Both boxes reach the slab about a tenth of a second after the start. From
then on the two halves of the table diverge.

The single-point box reports one contact, and that contact hops between the
corners of its bottom face from one time step to the next. The box never comes
to rest: it rocks at about 0.23 rad/s for the whole run, creeps sideways and
in yaw by a few degrees over several seconds, and sinks into the slab. Its
center, which would sit 20.00 mm above the slab at rest, reads 16.3 mm after
one second and 14.5 mm after five, so the box has sunk 5.5 mm into a slab it
should be resting on. (The contact stiffness alone would account for a few
micrometers of penetration under the box's 5 N weight.) In Meshcat the single
force arrow jumps around the bottom face.

The multipoint box reports four contacts, one at each corner of its bottom
face, and rests at exactly 20.00 mm with zero angular velocity from the moment
it lands. Meshcat shows four steady arrows.

The size of the single-point artifact depends on the time step; the manifold
result does not. With `--time_step=0.01` the single-point box falls straight
through the slab within half a second while the multipoint box still rests on
it. With `--time_step=0.0001` the single-point box settles to within 0.01 mm of
its rest height but still wobbles at 0.02 to 0.04 rad/s.

Why one point is not enough: a level box supported at a single point is in
equilibrium only if that point lies directly under its center of mass. The
deepest point of a face-face overlap is a corner (whichever one a tiny
numerical tilt favors), so the normal force there tips the box toward the
opposite corner, which then becomes the deepest point, and the box rocks from
corner to corner. Each exchange of support corners lets the center of mass
fall a little before the contact catches it, so the box ratchets downward,
and a larger time step makes each exchange coarser and the sinking faster.
Four contact points spanning the overlap region support the box the way a
table does.
