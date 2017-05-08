#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/mobilizer_impl.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

// Forward declarations.
template <typename T> class MultibodyTree;

namespace drake {
namespace multibody {

template <typename T>
class RevoluteMobilizer : public MobilizerImpl<T, 1, 1> {
 public:
  // Creates a revolute mobilizer with axis_F expressed in the inboard frame F.
  RevoluteMobilizer(const Frame<T>& inboard_frame,
                    const Frame<T>& outboard_frame,
                    const Vector3<double> axis_F) :
      MobilizerBase(inboard_frame, outboard_frame), axis_F_(axis_F) {}

  const Vector3<double>& get_revolute_axis() const { return axis_F_; }

 private:
  typedef MobilizerImpl<T, 1, 1> MobilizerBase;
  using MobilizerBase::nq;
  using MobilizerBase::nv;

  // Default joint axis expressed in the inboard frame F.
  Vector3<double> axis_F_;
};

}  // namespace multibody
}  // namespace drake
