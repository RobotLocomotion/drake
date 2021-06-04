#pragma once

#include "drake/common/drake_copyable.h"

namespace drake {
namespace multibody {
namespace internal {

/* An interface class for external auxiliary MultibodyPlant components that
 provides methods to query whether a component is scalar convertible to a
 particular scalar type.
 @tparam_default_scalar */
template <typename T>
class ScalarConvertibleComponent {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ScalarConvertibleComponent);

  ScalarConvertibleComponent() = default;

  virtual ~ScalarConvertibleComponent() = default;

  /* Returns true if `this` component can be scalar converted to double. */
  virtual bool is_cloneable_to_double() const = 0;

  /* Returns true if `this` component can be scalar converted to AutoDiffXd. */
  virtual bool is_cloneable_to_autodiff() const = 0;

  /* Returns true if `this` component can be scalar converted to
   symbolic::Expression. */
  virtual bool is_cloneable_to_symbolic() const = 0;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
