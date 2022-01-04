The SDFormat Composition mechanism
==================================

This README describes the use of [model composition in SDFormat](http://sdformat.org/tutorials?tut=composition_proposal&cat=pose_semantics_docs&)
in favor of the Model Directives mechanism.

A good start to review how it is being used, would be comparing the various
Model Directives files with their SDFormat counterpart used in
`test/model_directives_to_sdf_parity_test.cc`. These test files are in
`test/process_model_directives_test/*.sdf`.

Below are some features of Model Composition that allow us to achieve this, as
well as the caveats.

---

# Features

## Stricter scoping

Backreferences will not be allowed, for example in [`add_backreference.yaml`](test/process_model_directives_test/add_backreference.yaml).

More information about frame semantics and scopes can be found [here](http://sdformat.org/tutorials?tut=pose_frame_semantics_proposal&cat=pose_semantics_docs&).

## Merge including

This allows users to include `model`s described in a file into a top level
`world`/`model` without introducing an additional scope. For example,

in the downstream model,
```
<model name="to_be_included">
  <model name="model_1">
    ...
  </model>
  <model name="model_2">
    ...
  </model>
</model>

```
in the top level model,
```
<model name="top_level_model">
  <include merge="true">
    <uri>to_be_included</uri>
  </include>
</model>
```

This produces the scopes `top_level_model::model_1` instead of
`top_level_model::to_be_included::model_1`.

## Placement frames and fixed joints

Using a combination of `//include/placement_frame`, `//model/@placement_frame`,
`//pose/@relative_to` and fixed joints, users are able to recreate the bahavior
of `add_weld` from Model Directives. For example,


In Model Directives,
```
directives:
- add_model:
    name: simple_model
    file: simple_model.sdf
- add_model:
    name: extra_model
    file: simple_model.sdf
- add_weld:
    parent: simple_model::frame
    child: extra_model::base
```

In SDFormat it would be described as,
```
<model name="top_level">
  <include>
    <name>simple_model</name>
    <uri>simple_model.sdf</uri>
  </include>
  <include>
    <name>extra_model</name>
    <uri>simple_model.sdf</uri>
    <placement_frame>base</placement_frame>
    <pose relative_to="simple_model::frame"/>
  </include>
  <joint name="weld" type="fixed">
    <parent>simple_model::frame</parent>
    <child>extra_model::base</child>
  </joint>
</model>
```

---

# Caveats

* Adding scopes to names explicitly is no longer supported. Workflows that
  involve adding a frame (e.g. `left::arm::gripper_origin`) from a top level
  include would be impacted, while leveraging the top level scope (e.g. `left`)
  for other purposes, would be impacted and will need to be readjusted if
  SDFormat is used.

* Due to the stricter scoping rules, references to frames outside/above the
  current scope will no longer be allowed. Depends on the use-case, this
  can be circumvented using a combination of `//include/placement_frame`,
  `//include/pose` and `//include/pose/@relative_to` when including the model
  in the top level world/model. For example, in this directive where
  world is not defined in this model instance,

```
directives:
- add_model_instance:
    name: nested
- add_frame:
    name: nested::simple_model_origin
    X_PF:
      base_frame: world
      translation: [10, 0, 0]
  ...
```

```
<world name="top_level">
  <include>
    <uri>nested</uri>
    <placement_frame>simple_model_origin</placement_frame>
    <pose relative_to="world"/>
  </include>
</world>
```

* SDFormat files without a top level `actor`, `model` or `world` are not
  supported, therefore workflows that involve files describing a single weld
  would not be allowed. For example, `add_backreference.yaml`,

```
directives:
- add_weld:
    parent: simple_model_origin
    child: simple_model::base
```

* There is no support for including or merge-including a `world` into another
  `world`, whereas Model Directives do not have a strict top level limit.

---

## To be supported

* Allow merge including `model` into `world` where possible. If the included
  `model` contains elements that are not supported in a `world`, an error will
  be returned during parsing.

* [Reintroduce world joints](https://github.com/ignitionrobotics/sdf_tutorials/commit/52795e17bc6fb398b56801f94cb2cbf197c61d9f).
