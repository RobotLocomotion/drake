The Model Directives mechanism
==============================

"Model Directives" is a small YAML-based language for building a complex
MultibodyPlant-based scene out of numerous `.sdf` and `.urdf` files.  For
instance in the TRI dish-loading demo we have individual SDFormat files for
the counter, sink, cameras, pedestal, arm, gripper, and each manipuland.  A
single SDFormat for this would be unwieldy and difficult to maintain and
collaborate on, but SDFormat's file inclusion mechanisms have not yet proven
adequate to this task.

We expect that this mechanism will be temporary and will be removed
[when SDFormat adds similar functionality](#conditions-for-deprecation).
Users should be aware that this library will be deprecated if/when sdformat
reaches feature parity with it.


## Syntax

The easiest syntax reference is the unit test files in
`test/process_model_directives_test/*.yaml` of this directory.

A model directives file is a YAML file with a top level `directives:` group.
Within this group are a series of directives, which are described in detail in
`model_directives.h`.  These directives can add model instances and add frames
to them and welds between them.


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

Or, if you are connecting a scene graph:

```cpp
  DiagramBuilder builder;
  ModelDirectives station_directives = LoadModelDirectives(
      FindResourceOrThrow("my_exciting_project/models/my_scene.yaml"));
  auto [plant, scene_graph] =
      AddMultibodyPlantSceneGraph<double>(&builder, 0.0);;
  ProcessModelDirectives(station_directives, &plant);
  plant.Finalize();
```

This loads the model directives from that filename, constructs a plant, and
uses the model directives to populate the plant.


## Scoping

Elements (frames, bodies, etc.) in `MultibodyPlant` belong to model instances.
To acknowledge this, and support "faux" composition, we introduce scoping via
names.

Names can be scoped implicitly or explicitly, and this scoping supports
multiple levels of "nesting" -- insofar as model instances in MultibodyPlant
can denote hierarchy (see #14043 for more info).

Names are delimited by `::`; the last name portion indicates the element name,
and all preceding portions indicate the model instance name. For this reason,
element names themselves should not contain `::`.

(The `::` convention is used for consistency with the emerging namespace
syntax for SDFormat, soon to be required in SDFormat 1.8.)

Thus:

- `my_frame`: No explicit model instance, the frame is `my_frame`.
- `my_model::my_frame`: The model instance is `my_model`, the frame is
  `my_frame`.
- `top_level::my_model::my_frame`: The model instance is
  `top_level::my_model`, the frame is `my_frame`.


## Conditions for deprecation

We expect and hope to deprecate this mechanism when either:

SDFormat properly specifies, and Drake supports, the following:

 * What `<include/>` statements should *really* do (e.g. namespacing models,
   joints, etc.) without kludging Drake's parsing
   * See <http://sdformat.org/tutorials?tut=composition_proposal&cat=pose_semantics_docs&> for the progress of the proposed formal semantics.
 * How to weld models together with joints external to the models


OR if we find a mechanism whereby xacro could accomplish the same thing:

 * Drake's `package://` / `model://` mechanism were mature and correct, and
   `sdformat` didn't use singletons for search paths.
 * It was easier for one xacro to locate other xacros (possibly via a
   workaround using a wrapper script to inject the appropriate drake source
   or resource path)
