#pragma once

#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace valkyrie {

// All the following assumes using:
// urdf/urdf/valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf
// TODO(siyuan.feng): Add functinos to auto generate these.
DRAKE_DEPRECATED("2020-02-01", "The valkyrie example is being removed.")
constexpr int kRPYValkyrieDof = 36;

DRAKE_DEPRECATED("2020-02-01", "The valkyrie example is being removed.")
VectorX<double> RPYValkyrieFixedPointState();

// The nominal torque is generated with the QP controller at the setpoint,
// with acceleration constrained to zero.
DRAKE_DEPRECATED("2020-02-01", "The valkyrie example is being removed.")
VectorX<double> RPYValkyrieFixedPointTorque();

}  // namespace valkyrie
}  // namespace examples
}  // namespace drake

