#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace deformable {

/* We create a leaf system that outputs the desired state of a parallel jaw
 gripper to follow a close-lift-open motion sequence. The desired position is
 2-dimensional with the first element corresponding to the wrist degree of
 freedom and the second element corresponding to the left finger degree of
 freedom. This control is a time-based state machine, where desired state
 changes based on the context time. There are four states, executed in the
 following order:

  0. The fingers are open in the initial state.
  1. The fingers are closed to secure a grasp.
  2. The gripper is lifted to a prescribed final height.
  3. The fingers are open to loosen the grasp.

 The desired state is interpolated between these states. */
class ParallelGripperController : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ParallelGripperController);

  /* Constructs a ParallelGripperController system with the given parameters.
   @param[in] open_width   The width between fingers in the open state. (meters)
   @param[in] closed_width The width between fingers in the closed state.
                           (meters)
   @param[in] height       The height of the gripper in the lifted state.
                           (meters) */
  ParallelGripperController(double open_width, double closed_width,
                            double height);

 private:
  /* Computes the output desired state of the parallel gripper. */
  void CalcDesiredState(const systems::Context<double>& context,
                        systems::BasicVector<double>* output) const;

  /* The time at which the fingers reach the desired closed state. */
  const double fingers_closed_time_{1.5};
  /* The time at which the gripper reaches the desired "lifted" state. */
  const double gripper_lifted_time_{3.0};
  const double hold_time_{5.5};
  /* The time at which the fingers reach the desired open state. */
  const double fingers_open_time_{7.0};
  Eigen::Vector2d initial_configuration_;
  Eigen::Vector2d closed_configuration_;
  Eigen::Vector2d lifted_configuration_;
  Eigen::Vector2d open_configuration_;
};

}  // namespace deformable
}  // namespace examples
}  // namespace drake
