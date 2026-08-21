---
title: Troubleshooting common problems
layout: page_with_toc
---

{% include toc.md %}
<article class="markdown-body" markdown="1">

This page contains a collection of tips and tricks for resolving common
problems.

# MultibodyPlant

## Parsing {#mbp-parsing}

If you have a model file (e.g., .urdf, .sdf, .xml, etc.) that you are loading into a `MultibodyPlant` using the `Parser` class, and the model is not loading properly, then there are a few useful resources you might consider.

Documentation for the specific XML elements and attributes supported by the parser can be found [here](https://drake.mit.edu/doxygen_cxx/group__multibody__parsing.html).

If you do run into some of the known limitations in Drake's parsing support, you may consider using the [`make_drake_compatible_model`](https://manipulation.mit.edu/python/make_drake_compatible_model.html) tool to convert your model file and assets into a format that Drake can parse. It is available in the [manipulation](https://manipulation.mit.edu/) Python package, and be used via e.g.:
```bash
python3 -m venv venv
venv/bin/pip install manipulation[mesh] --extra-index-url https://drake-packages.csail.mit.edu/whl/nightly/
venv/bin/python -m manipulation.make_drake_compatible_model {input_file} {output_file}
```

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

## Loops in the body-joint graph {#mbp-loops-in-graph}

Currently Drake does not support automatic modeling of systems for which the
bodies and joints form one or more loops in the system graph. However, there
are several ways you can model these systems in Drake:

  - Break each loop at a joint and replace the joint by an equivalent constraint
    or by using a `LinearBushingRollPitchYaw`
    ([C++][c_LinearBushingRollPitchYaw],
    [Python][p_LinearBushingRollPitchYaw]) force element.
  - Break each loop by cutting one of the bodies involved in the loop. This
    introduces a new "shadow" body that will follow the "primary" body. For best
    numerical behavior, distribute the mass and inertia 1/2 to each of the
    two bodies. Use a Weld constraint to attach the shadow to its primary. This
    method has the advantage that joints remain uniformly treated and the
    joint coordinates are unchanged.

It is also possible to introduce loops by accident. If you didn't intend to
do so, check your system description to see whether some joint may have been
connected to the wrong body.

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

<!-- Remove Simulator section (and publish every time step) on 2026-06-01 when
deprecation is removed. -->

# Simulator

## Force publishing

`Simulator` had APIs to exercise its diagram's [force][force_trigger] events.
This includes

  - [`Simulator::set_publish_every_time_step()`][sim_every_step]
  - [`SimulatorConfig::publish_every_time_step`][sim_config_every_step]
  - The optional command-line parameter `simulator_publish_every_time_step`
  - [`Simulator::set_publish_at_initialization()`][sim_init]

These have all been deprecated (for removal on 2026-06-01). We don't expect this
will generally impact users. This guide will help those who may rely on this
functionality transition into the recommended mechanisms.

Rather than configuring an *entire* diagram to publish at initialization or
at each time step, each `LeafSystem` articulates independently whether it has
work that should be done at initialization or at each time step. This is done
by declaring [events][events] in each `LeafSystem`'s constructor.

### `Simulator::set_publish_every_time_step()`

To configure a LeafSystem to publish at every time step, use
[`LeafSystem::DeclarePerStepPublishEvent()`][c_LeafSystem_per_step_publish] in
the system's constructor.

This will most likely be defined for some form of system introspection: e.g.,
logging system state, broadcasting state to an external process. Less likely
would be visualization because an appropriate visualization rate is typically at
a lower frequency than that of simulation steps. Furthermore, well articulated
logging systems will typically be parameterized to control their publication
rate.

Note: introducing a per-step publish event implicitly includes publishing at
initialization.

There is also a deprecated [`Simulator::get_publish_every_time_step()`][get_publish_every_step], the
getter for this `Simulator` parameter. This will need to be evaluated on a per-leaf-system basis.

### `SimulatorConfig::publish_every_time_step`

This flag value was simply passed to `Simulator::set_publish_every_time_step()`.
If you find you actually have a leaf system that may need to publish at each
time step and that determination needs to be made at runtime, you'll have to
find an alternative means to configure it. You can create your own configuration
struct and include it in your scene configuration yaml.

### Command-line parameter `simulator_publish_every_time_step`

This command-line flag simply set the value of
`SimulatorConfig::publish_every_time_step`. Please refer to the previous section
for guidance.

### `Simulator::set_publish_at_initialization()`

To configure a LeafSystem to publish at every time step, use
[`LeafSystem::DeclareInitializationPublishEvent()`][c_LeafSystem_init_publish]
in the system's constructor.

Note: if your leaf system already includes a per-step publish event, you won't
also need an initialization publish event. Initialization is included in
per-step events.
<!-- 2026-06-01 End of block to delete. -->

# PyPI (pip)

## No candidate version for this platform {#pip-no-candidate}

<!-- Even though this is only relevant to past versions of Drake that are no
 longer supported, we'll keep this text around to help users of those older
 versions. -->

When installing Drake from PyPI on older platforms such as Ubuntu Focal, you may
receive the error "no candidate version for this platform". This is caused by
older versions of `pip` which do not recognize the `manylinux` platform used by
Drake. This is remedied by installing a newer version of `pip`.

Use of a Python virtual environment may be required in order to get a newer
version of `pip`, e.g.:

```bash
python3 -m venv env
env/bin/pip install --upgrade pip
env/bin/pip install drake
source env/bin/activate
```

# Image rendering

## GL and/or DISPLAY {#gl-init}

When performing image rendering (i.e., camera simulation), sometimes you may
need to configure your computer to provide Drake sufficient access to core
graphics libraries.

Drake renders images using the
[RenderEngine](https://drake.mit.edu/doxygen_cxx/classdrake_1_1geometry_1_1render_1_1_render_engine.html)
abstract base class, which is typically configured using the
[CameraConfig](https://drake.mit.edu/doxygen_cxx/structdrake_1_1systems_1_1sensors_1_1_camera_config.html)
data structure via YAML, which can specify a concrete `RenderEngine` subclass to
be used. Refer to the
[hardware_sim](https://github.com/RobotLocomotion/drake/tree/master/examples/hardware_sim)
example for details.

If you are using either `RenderEngineGl`, or `RenderEngineVtk` under the
(non-default) setting `backend = "GLX"`, then you must ensure that prior to
using Drake the `$DISPLAY` environment variable is set to an available X11
display server (e.g., `":1"`). If you are running as desktop user (not over
ssh), then `$DISPLAY` will probably already be set correctly.

For remote rendering (e.g., in the cloud) we recommend avoiding needing any
`$DISPLAY` by using only `RenderEngineVtk` and only with its default `backend`.
If you do need a display in the cloud, you'll need to run a program such as
`PyVirtualDisplay`, `Xvfb`, or a full `Xorg` server to provide it.

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

# Pydrake out of memory {#pydrake-oom}

Pydrake programs often control C++ objects that use a lot of memory, so that
the default behaviors for running garbage collection don't effectively control
memory growth.

The standard Python [gc](https://docs.python.org/3/library/gc.html) module
offers ways to customize the garbage collector's behavior. The easiest way is
to explicitly call ``gc.collect()`` at intervals in your program. This will
perform a full garbage collection. A more sophisticated method is to use
``gc.set_threshold()`` to set custom object count thresholds for your program,
and thus cause automatic collections to run more often.

See also: [Memory management with the Python Bindings](/python_bindings.html#memory-management-with-the-python-bindings)

# Network Configuration

## LCM on macOS {#lcm-macos}

When building and testing Drake from source on macOS 15 (Sequoia), you may
encounter issues with [LCM](https://lcm-proj.github.io/lcm/index.html).
In particular, an error message that can arise is: "LCM self test failed!!
Check your routing tables and firewall settings." LCM relies on
[UDP Multicast](https://lcm-proj.github.io/lcm/content/multicast-setup.html)
over loopback, so multicast traffic must be enabled over the loopback
interface on your computer in order for it to work.

Sometimes this is not explicitly enabled on macOS. If you see this error,
check the routing table by running `netstat -nr`. The following entry in the
IPv4 table is correct:

```
Internet:
Destination        Gateway            Flags               Netif Expire
...
224.0.0/4          lo0                UmS                   lo0
```

If you see a different interface for this address, such as `en0`, then
run the following to change it to loopback (`lo0`):

```
sudo route -nv delete 224.0.0.0/4
sudo route -nv add -net 224.0.0.0/4 -interface lo0
```

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
[c_LinearBushingRollPitchYaw]: https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_linear_bushing_roll_pitch_yaw.html
[p_LinearBushingRollPitchYaw]: https://drake.mit.edu/pydrake/pydrake.multibody.tree.html#pydrake.multibody.tree.LinearBushingRollPitchYaw
[c_MultibodyPlant]: https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_plant.html
[p_MultibodyPlant]: https://drake.mit.edu/pydrake/pydrake.multibody.plant.html#pydrake.multibody.plant.MultibodyPlant

<!-- drake/system -->
[events]: https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_event.html

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
[c_LeafSystem_per_step_publish]: https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_leaf_system.html#a49a07c6bbccc4464d5d6192889c3d2e6
[c_LeafSystem_init_publish]: https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_leaf_system.html#aab3136bba7eb6480a84309d019b28d83

<!-- Links to be removed upon 2026-06-01 deprecation removal. -->
[force_trigger]: https://drake.mit.edu/doxygen_cxx/namespacedrake_1_1systems.html#a59b7f49353f2a99b6c22d2eaae0fe9e9af8ece195be5dd5e820bdeee7ad21a4bf
[sim_every_step]: https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_simulator.html#aef1dc6aeb821503379ab1dd8c6044562
[get_publish_every_step]: https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_simulator.html#ae66775683e61fc461dec4f76bb8e5c7a
[sim_init]: https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_simulator.html#ac210a235b5e0865efb51fdd27c4b58ae
[sim_config_every_step]: https://drake.mit.edu/doxygen_cxx/structdrake_1_1systems_1_1_simulator_config.html#af1d9089360c8cd472de8f923ba7df99a
<!-- End of links to be removed.  -->
