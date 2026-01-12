#pragma once

#include <memory>
#include <utility>
#include <variant>
#include <vector>

#include "drake/multibody/contact_solvers/contact_solver_results.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/plant/discrete_contact_data.h"
#include "drake/multibody/plant/discrete_contact_pair.h"
#include "drake/multibody/plant/geometry_contact_data.h"
#include "drake/multibody/tree/acceleration_kinematics_cache.h"

namespace drake {
namespace multibody {
namespace internal {

/* When MbP is operating in "sampled" mode (vs "live") mode, this class will be
stored as State in the Context as an output of the discrete step, and will be
used to compute dynamics-related output ports. (When operating in "live" mode,
including continuous-time mode, this class is not used.)

The values here are populated during DiscreteUpdateManager::CalcDiscreteValues,
except for `reaction_forces` which is populated in the MbP.

Once it has been populated, the stored Data is immutable forever after. Copying
or assigning the `DiscreteStepMemory` just switches which immutable Data it is
pointing to, it doesn't overwrite anything. */
struct DiscreteStepMemory {
  /* Because DiscreteStepMemory lives as abstract state in the context, it can't
  be templated on a scalar type. Instead, we'll use a templated nested struct,
  with DiscreteStepMemory holding variant of the allowed types. This is also
  provides a nice place to put our shared_ptr-const wrapper to implement cheap
  copying of large, immutable data. */
  template <typename T>
  struct Data {
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Data);

    /* The constructor requires the topology to pre-size things. */
    explicit Data(const internal::SpanningForest& forest);

    ~Data();

    VectorX<T> net_actuation;
    GeometryContactData<T> geometry_contact_data;
    AccelerationKinematicsCache<T> acceleration_kinematics_cache;

    // TODO(jwnimmer-tri) This structure is probably more than we really need.
    // We may only need the list of X_WC transforms, not the full kinematics.
    DiscreteContactData<DiscreteContactPair<T>> discrete_contact_pairs;

    contact_solvers::internal::ContactSolverResults<T> contact_solver_results;

    // N.B. This field is populated by MultibodyPlant::CalcStepUnrestricted, not
    // the DiscreteUpdateManager::CalcDiscreteValues.
    std::vector<SpatialForce<T>> reaction_forces;
  };

  /* Resets this object with fresh-allocated (empty) data.
  Returns a mutable reference to the new data.
  @tparam_default_scalar */
  template <typename T>
  Data<T>& Allocate(const internal::SpanningForest& forest);

  /* If this memory holds data for scalar type T, then returns a const pointer
  to the data. Otherwise, returns nullptr. */
  template <typename T>
  const Data<T>* get() const {
    if (auto* maybe_data = std::get_if<std::shared_ptr<const Data<T>>>(&data)) {
      return maybe_data->get();
    }
    return nullptr;
  }

  // We use variant because abstract state in the context cannot template on T.
  // We use shared_ptr-to-const so that we're cheap to copy and move.
  std::variant<std::shared_ptr<const Data<double>>,
               std::shared_ptr<const Data<AutoDiffXd>>,
               std::shared_ptr<const Data<symbolic::Expression>>>
      data;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
