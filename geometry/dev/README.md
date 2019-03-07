# SceneGraph with Rendering

This dev folder contains a stopgap solution to provide functionality
while the full functionality works its way into master's mainstream
code base (See
[Issue 9540](https://github.com/RobotLocomotion/drake/issues/9540).)

The functionality is rendering functionality -- supporting simultaneous
simulation and perception simulation with the
`MultiBodyPlant`-`SceneGraph` workflow. The contents of this folder
should be treated as a temporary stopgap. Its API should *not* be
considered the final API -- the API in this folder is an inglorious
combination of the initial design pass constrained by the exigencies of
making it available at all. Even while the aforementioned issue 9540
is working its way through, this folder may or may not change. That has
not been decided.

But even with its current limitations, this folder provides sufficient
functionality to produce color, depth, and label images from MBP.

## Design issues

### Relationship between `geometry` and `geometry::dev`

This folder introduces an *additional* class named `SceneGraph`. They
are differentiated by namespace (e.g., `geometry::SceneGraph` and
`geometry::dev::SceneGraph`). (The name duplication is an anachronism
arising from an early pass in which the two were intended to be used
interchangeably -- an attempt that has since been abandoned, leaving
this name residue.) While the two `SceneGraph` implementations have much
in common, they are *not* equivalent.

The original `geometry::SceneGraph` will be the ultimate recipient of
the final version of all the rendering functionality found in
`geometry::dev::SceneGraph`. At that point, this dev folder will be
removed. Currently, however, it *only* supports proximity queries.

The new `geometry::dev::SceneGraph`, however, only supports *rendering*
queries. The two SceneGraph instances are intended to be used in a
complementary way. A single diagram can include both types -- one to
support proximity queries (aka collision) and the other can support
rendering.

Both `SceneGraph` types can support external visualization and there
is an overloaded function `ConnectDrakeVisualizer()` (one in `geometry`
and one in `geometry::dev`) that can work on either `SceneGraph` type.

### Populating the `geometry::dev::SceneGraph`

To facilitate standing this dev folder up, the
`geometry::dev::SceneGraph` is not wholly compatible with the
`geometry::SceneGraph` ecosystem. In more concrete terms, parsing can
only populate a `geometry::SceneGraph`. That means, if the MBP and a
`SceneGraph` is to be populated from SDF files, the user *must*
instantiate an instance of `geometry::SceneGraph` (even if the
application only wants to use the `geometry::dev::SceneGraph` for
rendering). The latter is constructed from the former.

Because `geometry::dev::SceneGraph` only supports rendering, the
conversion from `geometry::SceneGraph` to `geometry::dev::SceneGraph`
*may* omit some of the registered geometries. Specifically, any geometry
registered with a visual material whose diffuse value has a zero-valued
*alpha* channel will not be transferred to the
`geometry::dev::SceneGraph`. (MBP uses this as a way to make sure
geometry explicitly registered as *collision* geometry does not clutter
up the visualization -- this dev folder exploits that convention).

## Workflow

### Proximity Query Only

Simply use the normal workflow used in a multitude of examples. See
`drake/examples/scene_graph/dev/bouncing_ball*` as an example.

### Rendering Query Only

1. Instantiate `geometry::SceneGraph` but *don't* add it to the diagram
   builder.
2. Perform all parsing/geometry registration with *this* instance of
   `SceneGraph`.
3. Once everything is loaded, add a `geometry::dev::SceneGraph` to the
   diagram, passing the `geometry::SceneGraph` instance into the
   constructor.
4. Connect the query object output port of the
   `geometry::dev::SceneGraph` instance to the input port of an object
   that guarantees only to make rendering queries (e.g.,
   `systems::sensors::dev::RgbdCamera`).
5. In all other ways, connect the `geometry::dev::SceneGraph` to MBP
   ports, visualization, etc.
6. Throw away the original `geometry::SceneGraph`.

NOTE: If you are *not* parsing SDF files, you can instantiate the
`geometry::dev::SceneGraph` directly and populate that through the
normal registration process. See
`drake/examples/scene_graph/dev/solar_system*` as an example. In doing
so, the visual material still matters -- if the alpha value is non-zero,
the geometry will be included for both rendering and external
visualization. If it is zero, the geometry will functionally be
non-existant.

### Proximity and Rendering Queries

The same as _Rendering Query Only_ except add the original
`geometry::SceneGraph` to the diagram. MBP will need to be connected to
*both* `SceneGraph` types. Only one of them needs to be connected to
visualization (connecting the `geometry::dev::SceneGraph` would be
preferred as it will *not* broadcast the collision geometry to the
visualizers as invisible geometry).

See `drake/examples/scene_graph/dev/bouncing_ball_run_dynamics.cc` for
an example of a diagram that has *both* `SceneGraph` types. The
`geometry::dev::SceneGraph` is used for driving external visualization
as well as providing the render queries for an instance of
`systems::sensors::dev::rgbd_camera`. It also exports the rendered
images out to LCM. To see the rendered images run `drake_visualizer` as:

```
./bazel-bin/tools/drake_visualizer --script systems/sensors/visualization/show_images.py
```

## Known issues

- The conversion from `geometry::SceneGraph` to
  `geometry::dev::SceneGraph` has *not* been rigorously tested. Sampling
  suggests its doing the *right* thing, but there's no proof that's
  generally the case.
- `geometry::dev::QueryObject` still has the vestiges of proximity
  queries; however, calling them will cause runtime exceptions to be
  thrown.
