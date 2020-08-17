The Model Directives mechanism
==============================

Model Directives is a small yaml-based language for building a complex
MultibodyPlant-based scene out of numerous SDFs.  For instance in the TRI
dish-loading demo we have individual SDF files for the counter, sink, cameras,
pedestal, arm, gripper, and each manipuland.  A single SDF for this would be
unwieldy and difficult to maintain and collaborate on, but SDF's file
inclusion mechanisms have not yet proven adequate to this task.

We expect that this mechanism will be temporary and will be removed when
sdformat adds similar functionality.  Users should be aware that this library
will be deprecated if/when sdformat reaches feature parity with it.


## Syntax

The easiest syntax reference is the unit test files in `test/models/*.yaml` of
this directory.

A model directives file is a yaml file with a top level `directives:` group.
Within this group are a series of directives:

 * `AddModel` takes a `file` and `name` and loads the SDF/URDF file indicated
   as a new model instance with the given name.
 * `AddModelInstance` Creates a new, empty model instance in the plant with
   the indicated `name`.
 * `AddPackagePath` takes `name` and `path` and makes `package://name` URIs
   be resolved to `path`.  This directive is due for deprecation soon and
   should generally be avoided.
 * `AddFrame` takes a `name` and a `X_PF` transform and adds a new frame to
   the model.  Note that the transform is as specified in the `Transform`
   scenario schema and can reference an optional base frame, in which case
   the frame will be added to the base frame's model instance.
 * `AddDirectives` takes a `file` naming a model directives file and an
   optional `model_namespace`; it loads the model directives from that file
   with the namespace prefixed to them (see Scoping, below).
 * `AddWeld` takes a `parent` and `child` frame and welds them together.


## Use

The easiest use reference is the unit test `process_model_directives_test.cc`.

A simple example of a use would be:

```cpp
  ModelDirectives station_directives = LoadModelDirectives(
      FindResourceOrThrow("my_exciting_project/models/my_scene.yaml"));
  MultibodyPlant<double> plant;
  ProcessModelDirectives(station_directives, &plant);
  plant.Finalize();
```

This loads the model directives from that filename, constructs a plant, and
uses the model directives to populate the plant.


## Scoping

Elements (frames, bodies, etc.) in `MultibodyPlant` belong to model instances.
Model instances can have any name specifiers, and can contain the "namespace"
delimiter `::`. Element names should not contain `::`.

Examples:

- `my_frame` implies no explicit model instance.
- `my_model::my_frame` implies the model instance `my_model`, the frame
`my_frame`.
- `top_level::my_model::my_frame` implies the model instance
`top_level::my_model`, the frame `my_frame`.


## Conditions for deprecation

We expect and hope to deprecate this mechanism when either:

SDF format properly specifies, and Drake supports, the following:

 * What `<include/>` statements should *really* do (e.g. namespacing models,
   joints, etc.) without kludging Drake's parsing
 * How to weld models together with joints external to the models

OR if we find a mechanism whereby xacro could accomplish the same thing:

 * Drake's `package://` / `model://` mechanism were mature and correct, and
   `sdformat` didn't use singletons for search paths.
 * It was easier for one xacro to locate other xacros (possibly via a
   workaround using a wrapper script to inject `DRAKE_PATH`)
