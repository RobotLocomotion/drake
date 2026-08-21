#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#include <array>
#include <utility>

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

// Symbol: pydrake_doc_systems_rendering
constexpr struct /* pydrake_doc_systems_rendering */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::systems
    struct /* systems */ {
      // Symbol: drake::systems::rendering
      struct /* rendering */ {
        // Symbol: drake::systems::rendering::MultibodyPositionToGeometryPose
        struct /* MultibodyPositionToGeometryPose */ {
          // Source: drake/systems/rendering/multibody_position_to_geometry_pose.h
          const char* doc =
R"""(A direct-feedthrough system that converts a vector of joint positions
directly to a geometry∷FramePoseVector<T> to behave like a
MultibodyPlant∷get_geometry_pose_output_port().

.. pydrake_system::

    name: MultibodyPositionToGeometryPose
    input_ports:
    - position
    output_ports:
    - geometry_pose

The position input must be a vector whose length matches either the
number of positions in the MultibodyPlant or the number of states
(based on the optional argument in the constructor). This option to
pass the full state vector is provided only for convenience -- only
the position values will affect the outputs.)""";
          // Symbol: drake::systems::rendering::MultibodyPositionToGeometryPose::MultibodyPositionToGeometryPose<T>
          struct /* ctor */ {
            // Source: drake/systems/rendering/multibody_position_to_geometry_pose.h
            const char* doc_2args_plant_input_multibody_state =
R"""(The MultibodyPositionToGeometryPose holds an internal, non-owned
reference to the MultibodyPlant object so you must ensure that
``plant`` has a longer lifetime than ``this``
MultibodyPositionToGeometryPose system.

Parameter ``input_multibody_state``:
    If true, the vector input port will be the size of the ``plant``
    *state* vector. If false, it will be the size of the ``plant``
    *position* vector. In both cases, only the positions will affect
    the output. $*Default:* false.

Raises:
    if ``plant`` is not finalized and registered with a SceneGraph.)""";
            // Source: drake/systems/rendering/multibody_position_to_geometry_pose.h
            const char* doc_2args_owned_plant_input_multibody_state =
R"""(The MultibodyPositionToGeometryPose owns its internal plant.

Parameter ``input_multibody_state``:
    If true, the vector input port will be the size of the ``plant``
    *state* vector. If false, it will be the size of the ``plant``
    *position* vector. In both cases, only the positions will affect
    the output. @default: false.

Raises:
    if ``owned_plant`` is not finalized and registered with a
    SceneGraph.)""";
          } ctor;
          // Symbol: drake::systems::rendering::MultibodyPositionToGeometryPose::multibody_plant
          struct /* multibody_plant */ {
            // Source: drake/systems/rendering/multibody_position_to_geometry_pose.h
            const char* doc = R"""()""";
          } multibody_plant;
          // Symbol: drake::systems::rendering::MultibodyPositionToGeometryPose::owns_plant
          struct /* owns_plant */ {
            // Source: drake/systems/rendering/multibody_position_to_geometry_pose.h
            const char* doc =
R"""(Returns true if this system owns its MultibodyPlant.)""";
          } owns_plant;
        } MultibodyPositionToGeometryPose;
      } rendering;
    } systems;
  } drake;
} pydrake_doc_systems_rendering;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
