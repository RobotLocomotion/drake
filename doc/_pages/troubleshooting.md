---
title: Troubleshooting common problems
---

This page contains a collection of tips and tricks for resolving common
problems.

# MultibodyPlant

## Exception messages

### Unconnected QueryObject port {#mbp-unconnected-query-object-port}

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

<!-- Links to the various Drake doxygen pages.
     Order determined by directory structure first and names second.
-->

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
[p_GetMyContextFromRoot]: https://drake.mit.edu/pydrake/pydrake.systems.framework.html#pydrake.systems.framework.System_.System_[float].GetMyContextFromRoot
