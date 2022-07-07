---
title: Troubleshooting common problems
---

This page contains a collection of tips and tricks for resolving common
problems.

# MultibodyPlant

## Exception messages

### Unconnected [QueryObject][_QueryObject] port {#mbp-unconnected-query-object-port}

The error message will include the message, "The provided context doesn't show a connection for the plant's query input port."

[MultibodyPlant][_MultibodyPlant] requires a connection to
[SceneGraph][_SceneGraph] in order to evaluate contact. Attempts to evaluate
output ports which expose or depend on contact results will fail. Evaluating the
time derivatives (e.g., when running a continuous simulation) will likewise
fail.

The connection can be reported as missing for several reasons:

1. Was a SceneGraph instance created and connected?
   - The simplest solution is to use
     [AddMultibodyPlantSceneGraph()][_AddMultibodyPlantSceneGraph]. It will
     instantiate and connect the plant and scene_graph together for you.
2. Did you provide the right context?
   - You've created and connected the systems in a [Diagram][_Diagram] and
     allocated a [Context][_Context] for the Diagram.
   - Make sure you extract the MultibodyPlant's context from the Diagram's
     Context. Do not allocate a Context directly from the plant. Use
     [System::GetMyContextFromRoot()][_GetMyContextFromRoot]
     to acquire the plant's Context from the Diagram's context; this will
     preserve all of the necessary connections. E.g.,

Python
```py
  builder = DiagramBuilder()
  # Build plant and scene graph and automatically connect them.
  plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-3)
  ...
  diagram = builder.Build()
  context = diagram.CreateDefaultContext()
  # Extract the plant's context from the diagram's to invoke plant methods.
  plant_context = plant.GetMyContextFromRoot(context)
```

C++
```c++
  DiagramBuilder<double> builder;
  // Build plant and scene graph and automatically connect them.
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 1e-3);
  ...
  std::unique_ptr<Diagram<double>> diagram = builder.Build();
  std::unique_ptr<Context<double>> context = diagram->CreateDefaultContext();
  // Extract the plant's context from the diagram's to invoke plant methods.
  const Context<double>& plant_context = plant.GetMyContextFromRoot(*context);
```

[//]: # (Links to the various Drake doxygen pages.)
[//]: # (Order determined by directory structure first and names second.)

[//]: # (drake/geometry)
[_QueryObject]: https://drake.mit.edu/doxygen_cxx/classdrake_1_1geometry_1_1_query_object.html
[_SceneGraph]: https://drake.mit.edu/doxygen_cxx/classdrake_1_1geometry_1_1_scene_graph.html

[//]: # (drake/multibody/plant)
[_AddMultibodyPlantSceneGraph]: https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_plant.html#aac66563a5f3eb9e2041bd4fa8d438827
[_MultibodyPlant]: https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_plant.html

[//]: # (drake/systems/framework)
[_Context]: https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_context.html
[_Diagram]: https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_diagram.html
[_GetMyContextFromRoot]: https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_system.html#ae7fa91d2b2102457ced3361207724e52
