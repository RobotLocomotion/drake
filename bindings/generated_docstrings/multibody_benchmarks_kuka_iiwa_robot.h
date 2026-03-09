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

// #include "drake/multibody/benchmarks/kuka_iiwa_robot/make_kuka_iiwa_model.h"

// Symbol: pydrake_doc_multibody_benchmarks_kuka_iiwa_robot
constexpr struct /* pydrake_doc_multibody_benchmarks_kuka_iiwa_robot */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::multibody
    struct /* multibody */ {
      // Symbol: drake::multibody::benchmarks
      struct /* benchmarks */ {
        // Symbol: drake::multibody::benchmarks::kuka_iiwa_robot
        struct /* kuka_iiwa_robot */ {
          // Symbol: drake::multibody::benchmarks::kuka_iiwa_robot::MakeKukaIiwaModel
          struct /* MakeKukaIiwaModel */ {
            // Source: drake/multibody/benchmarks/kuka_iiwa_robot/make_kuka_iiwa_model.h
            const char* doc =
R"""(This method makes a MultibodyTree model for a Kuka Iiwa arm as
specified in the file kuka_iiwa_robot.urdf contained in this same
directory. Links can be accessed by their name "iiwa_link_1" (base)
through "iiwa_link_7" (end effector). The "world" body can be accessed
with MultibodyTree∷world_body(). Joints can be accessed by their name
"iiwa_joint_1" (from the base) through "iiwa_joint_7" (to the end
effector). The new MultibodyTree model is finalized by
MultibodyTree∷Finalize() and therefore no more modeling elements can
be added.

Parameter ``finalize_model``:
    If ``True``, the model is finalized with MultibodyTree∷Finalize().
    A non-finalized model can be requested if adding more multibody
    elements is desired.

Parameter ``gravity``:
    The value of the acceleration of gravity, in m/s².)""";
          } MakeKukaIiwaModel;
        } kuka_iiwa_robot;
      } benchmarks;
    } multibody;
  } drake;
} pydrake_doc_multibody_benchmarks_kuka_iiwa_robot;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
