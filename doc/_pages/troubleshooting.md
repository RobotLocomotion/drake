---
title: Troubleshooting common problems
layout: page_with_toc
---

{% include toc.md %}
<article class="markdown-body" markdown="1">

This page contains a collection of tips and tricks for resolving common
problems.

# MultibodyPlant

## Unconnected QueryObject port {#mbp-unconnected-query-object-port}

The error message will include the message, "The provided context doesn't show a
connection for the plant's query input port."

`MultibodyPlant` ([C++][c_MultibodyPlant], [Python][p_MultibodyPlant])
requires a connection to `SceneGraph` ([C++][c_SceneGraph],
[Python][p_SceneGraph]) in order to evaluate contact via its  `QueryObject`
([C++][c_QueryObject], [Python][p_QueryObject]). Attempts to evaluate output
ports that expose or depend on contact results (the geometric data and/or forces
that arise from the contact between bodies' collision geometries) will fail.
Evaluating the time derivatives (e.g., when running a continuous simulation)
will likewise fail.

The connection can be reported as missing for several reasons:

1. Was a `SceneGraph` instance created and connected?
   - The simplest solution is to use `AddMultibodyPlantSceneGraph()`
     ([C++][c_AddMultibodyPlantSceneGraph],
     [Python][p_AddMultibodyPlantSceneGraph]). It will construct and connect
     the plant and scene graph together for you.
2. Did you provide the right context?
   - You've created and connected the systems in a `Diagram`
     ([C++][c_Diagram], [Python][p_Diagram]) and allocated a `Context`
     ([C++][c_Context], [Python][p_Context]) for the `Diagram`.
   - Make sure you extract the `MultibodyPlant`'s context from the `Diagram`'s
     `Context`. Do not allocate a `Context` directly from the plant. Use
     `System::GetMyContextFromRoot()` ([C++][c_GetMyContextFromRoot],
     [Python][p_GetMyContextFromRoot]) to acquire the plant's `Context` from the
     `Diagram`'s context; this will preserve all of the necessary connections.
     E.g.,

    ```py
      builder = DiagramBuilder()
      # Build plant and scene graph and automatically connect them.
      plant, scene_graph = AddMultibodyPlantSceneGraph(
          builder=builder, time_step=1e-3)
      ...
      diagram = builder.Build()

      # WRONG.  This will create a Context for the plant only (no connected
      # SceneGraph).
      xdot = plant.EvalTimeDerivatives(context=plant.CreateDefaultContext())

      context = diagram.CreateDefaultContext()
      # Extract the plant's context from the diagram's to invoke plant methods.
      plant_context = plant.GetMyContextFromRoot(root_context=context)
      xdot = plant.EvalTimeDerivatives(context=plant_context)
    ```

# System Framework

## Context-System mismatch {#framework-context-system-mismatch}

The error message will include one of the following:

  - `A function call on a FooSystem system named '::_::foo' was passed the root
     Diagram's Context instead of the appropriate subsystem context.`
  - `A function call on the root Diagram was passed a subcontext associated with
     its subsystem named '::_::foo' instead of the root context.`
  - `A function call on a BarSystem named '::_::bar' was passed the Context of
     a system named '::_::foo' instead of the appropriate subsystem Context.`

<!-- TODO(SeanCurtis-TRI): This overview of Systems and Contexts would be
 better in a doxygen module and just referenced here. Ping Russ to see if his
 textbook covers this material and if we can/should steal it. -->

<h4>Contexts and Systems: An Overview</h4>

For every `System` ([C++][c_System], [Python][p_System]) there is a
corresponding `Context` ([C++][c_Context], [Python][p_Context]). The `System`
and `Context` work together. The `Context` holds the data values and the
`System` defines the operations on that data.

For example, a `MultibodyPlant` ([C++][c_MultibodyPlant],
[Python][p_MultibodyPlant]) is such a system. We can use an instance of a
`MultibodyPlant` to create one or more `Context`s. Each `Context` contains the
continuous and discrete state values for the plant's model. These `Context`s
can be configured to represent the model in different configurations. However,
if we want to evaluate some mechanical property (e.g., composite inertia of a
robotic arm), we invoke a method on the plant, passing one of the contexts.

We can combine multiple `System`s into a `Diagram` ([C++][c_Diagram],
[Python][p_Diagram]). The `System`s within a `Diagram` will typically have their
ports connected -- this is how the `System`s work together.

`Context`s similarly form a parallel hierarchical structure. The `Context`
associated with a `Diagram` is the combination of all of the `Context`s
associated with the `System`s inside that `Diagram`. The port connections
between `System`s in the `Diagram` are mirrored in the `Diagram`'s `Context`.

<h4>Why did I get this error and how do I get rid of it?</h4>

Many `System` APIs require a `Context`. It is important to pass the *right*
`Context` into the API. What's the difference between a "right" and "wrong"
`Context`?

  - The right `Context` was allocated by the `System` being evaluated.
  - If the `System` is part of a `Diagram`, then the `Context` provided should
    be a reference pointing inside the `Diagram`s `Context` (see below for how
    to do this).

The most common error is to pass the `Diagram`'s `Context` into a constituent
`System`'s API. We'll illustrate this using a `Diagram` with a `MultibodyPlant`.
The solution is to use either `GetMyContextFromRoot()`
([C++][c_GetMyContextFromRoot], [Python][p_GetMyContextFromRoot]) or
`GetMyMutableContextFromRoot()` ([C++][c_GetMyMutableContextFromRoot], [Python]
[p_GetMyMutableContextFromRoot]) as appropriate to extract a particular
`System`'s `Context` from its `Diagram`'s `Context`.

```py
  builder = DiagramBuilder()
  plant, _ = AddMultibodyPlantSceneGraph(builder=builder, time_step=1e-3)
  ...  # Populate the plant with interesting stuff.
  diagram = builder.Build()
  # Create a Context for diagram.
  root_context = diagram.CreateDefaultContext()

  my_body = plant.GetBodyByName("my_body")
  X_WB = RigidTransform(...)  # Define a pose for the body.
  # Error! plant has been given diagram's context.
  plant.SetFreeBodyPose(context=root_context, body=base, X_WB=X_WB_desired)

  # Get the Context for plant from diagram's context.
  plant_context = plant.GetMyContextFromRoot(root_context=root_context)
  # Successful operation; the provided context belongs to plant.
  plant.SetFreeBodyPose(context=plant_context, body=base, X_WB=X_WB_desired)
```

See the notes on [System Compatibility][m_system_compat] for further discussion.

# Build problems

## Out of memory {#build-oom}

When compiling Drake from source, the build might run out of memory. The error
message will include the message, "cc: fatal error: Killed signal terminated
program cc1plus".

By default, the Drake build will try use all available CPU and RAM resources on
the machine. Sometimes, when there is not enough RAM per CPU, the build might
crash because it tried to run a compiler process on every vCPU and together
those compilation jobs consumed more RAM than was available.

In this case, you'll need to add a bazel configuration dotfile to your home
directory. Create a text file at `$HOME/.bazelrc` with this content:

```
build --jobs=HOST_CPUS*0.4
```

This instructs the build system to use only 40% of the available vCPUs. You may
tune the fraction up or down to balance build speed vs resource crashes. You may
also specify an exact number like `build --jobs=2` instead of a fraction.

The dotfile will affect any Bazel builds that you run. If you prefer to change
only the Drake build instead of all Bazel builds, you may place the dotfile in
the Drake source tree at `drake/user.bazelrc` instead of your home directory.

Note that the concurrency level passed to `make` (e.g., `make -j 2`) does not
propagate through to affect the concurrency of most of Drake's build steps; you
need to configure the dotfile in order to control the build concurrency.


<!-- Links to the various Drake doxygen pages.
     Order determined by directory structure first and names second.
-->

<!-- modules -->
[m_system_compat]: https://drake.mit.edu/doxygen_cxx/group__system__compatibility.html

<!-- drake/geometry -->
[c_QueryObject]: https://drake.mit.edu/doxygen_cxx/classdrake_1_1geometry_1_1_query_object.html
[p_QueryObject]: https://drake.mit.edu/pydrake/pydrake.geometry.html#pydrake.geometry.QueryObject_
[c_SceneGraph]: https://drake.mit.edu/doxygen_cxx/classdrake_1_1geometry_1_1_scene_graph.html
[p_SceneGraph]: https://drake.mit.edu/pydrake/pydrake.geometry.html#pydrake.geometry.SceneGraph_

<!-- drake/multibody/plant -->
[c_AddMultibodyPlantSceneGraph]: https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_plant.html#aac66563a5f3eb9e2041bd4fa8d438827
[p_AddMultibodyPlantSceneGraph]: https://drake.mit.edu/pydrake/pydrake.multibody.plant.html#pydrake.multibody.plant.AddMultibodyPlantSceneGraph
[c_MultibodyPlant]: https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_plant.html
[p_MultibodyPlant]: https://drake.mit.edu/pydrake/pydrake.multibody.plant.html#pydrake.multibody.plant.MultibodyPlant

<!-- drake/systems/framework -->
[c_Context]: https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_context.html
[p_Context]: https://drake.mit.edu/pydrake/pydrake.systems.framework.html#pydrake.systems.framework.Context
[c_Diagram]: https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_diagram.html
[p_Diagram]: https://drake.mit.edu/pydrake/pydrake.systems.framework.html#pydrake.systems.framework.Diagram
[c_GetMyContextFromRoot]: https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_system.html#ae7fa91d2b2102457ced3361207724e52
[p_GetMyContextFromRoot]: https://drake.mit.edu/pydrake/pydrake.systems.framework.html#pydrake.systems.framework.System_.System_[float].GetMyMutableContextFromRoot
[c_GetMyMutableContextFromRoot]: https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_system.html#ae7fa91d2b2102457ced3361207724e52
[p_GetMyMutableContextFromRoot]: https://drake.mit.edu/pydrake/pydrake.systems.framework.html#pydrake.systems.framework.System_.System_[float].GetMyMutableContextFromRoot
[c_System]: https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_system.html
[p_System]: https://drake.mit.edu/pydrake/pydrake.systems.framework.html#pydrake.systems.framework.System
