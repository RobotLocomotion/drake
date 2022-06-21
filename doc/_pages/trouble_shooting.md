---
title: Troubleshooting common problems
---

This page contains a collection of tips and tricks for resolving common
problems.

# MultibodyPlant

## Exception messages

### Unconnected QueryObject port {#mbp-unconnected-query-object-port}

The error message will include the message, "The provided context doesn't show a connection for the plant's query input port."

MultibodyPlant requires a connection to SceneGraph in order to evaluate contact.
Attempts to evaluate output ports which expose or depend on contact results
will fail. Evaluating the time derivatives (e.g., when running a continuous
simulation) will likewise fail.

The connection can be reported as missing for several reasons:

1. Was a [SceneGraph](https://drake.mit.edu/doxygen_cxx/classdrake_1_1geometry_1_1_scene_graph.html) instance created and connected?
   - The simplest solution is to use [AddMultibodyPlantSceneGraph()](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_plant.html#aac66563a5f3eb9e2041bd4fa8d438827). It will instantiate the plant and
   scene_graph together and also connect them for you.
2. Did you provide the right context?
   - You've created and connected the systems in a Diagram and allocated a
     Context for the Diagram.
   - Make sure you extract the MultibodyPlant's context from the Diagram's
     Context. Do not allocate a Context directly from the plant. Use
     [GetMyContextFromRoot()](https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_system.html#ae7fa91d2b2102457ced3361207724e52)
     to acquire the plant's Context from the Diagram's context; this will
     preserve all of the necessary connections. E.g.,

```c++
  MultibodyPlant<double>* plant = ...;
  std::unique_ptr<Diagram<double>> diagram = builder.Build();
  std::unique_ptr<Context<double>> context = diagram->CreateDefaultContext();
  Context<double>& plant_context = plant->GetMyContextFromRoot(*context);
```
