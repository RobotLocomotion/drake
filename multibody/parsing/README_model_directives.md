The Model Directives mechanism
==============================

"Model Directives" is a small YAML-based language for building a complex
MultibodyPlant-based scene out of numerous `.sdf` and `.urdf` files.  For
instance in the TRI dish-loading demo we have individual SDFormat files for
the counter, sink, cameras, pedestal, arm, gripper, and each manipuland.  A
single SDFormat for this would be unwieldy and difficult to maintain and
collaborate on, but SDFormat's file inclusion mechanisms have not proven
adequate to this task.

Model directives are a supported part of Drake's parsing. There are no current
plans to deprecate them. See [Relationship to SDFormat](#relationship-to-sdformat)
for the gaps that would have to close first.


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

Every use of `add_directives` with a `model_namespace` will append that
namespace to the model instance name while parsing the referenced `file`.  All
element names will be automatically prefixed with that model namespace, with
the exception of `add_frame` names without a model instance.

When `add_frame` has a model instance scope in its name, `add_frame` will
use that as the prefix all element names like any other directive would; if
no model instance is present, however, `add_frame` will check the associated
`X_PF.base_frame` for a model instance prefix and, if it is present, use it
as the prefix for the `name` element.


## Units

We use SI units plus radians for all physical quantities.

## Relationship to SDFormat

Replacing model directives with SDFormat was explored (see the Drake issue
"sdformat: Should attempt migration from model directives" and the closed
parity-test PR "Feature parity tests between sdformat and model directives")
and is not currently planned. Deprecation would require, at minimum:

 * Merge-including models into a `<world>` without introducing an extra model
   scope.
 * A way to weld models together using joints (or an equivalent) external to
   the models, at world level.
 * A story for the "leaky" scoping that directives allow but SDFormat's
   encapsulation forbids.

See <http://sdformat.org/tutorials?tut=composition_proposal&cat=pose_semantics_docs&>
for the proposed formal include semantics.

The same would hold for a xacro-based alternative, which would additionally
need:

 * Drake's `package://` / `model://` mechanism to be mature and correct, and
   `sdformat` to not use singletons for search paths.
 * It to be easier for one xacro to locate other xacros (possibly via a
   workaround using a wrapper script to inject the appropriate drake source
   or resource path)
