#pragma once

#include <string>
#include <typeinfo>

#include "drake/common/default_scalars.h"
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

  virtual ~ScalarConvertibleComponent();

  /* Returns true if `this` component can be scalar converted to double. */
  virtual bool is_cloneable_to_double() const = 0;

  /* Returns true if `this` component can be scalar converted to AutoDiffXd. */
  virtual bool is_cloneable_to_autodiff() const = 0;

  /* Returns true if `this` component can be scalar converted to
   symbolic::Expression. */
  virtual bool is_cloneable_to_symbolic() const = 0;

  /* (Optional, display-only) If this component blocks scalar conversion to the
   destination scalar identified by `scalar`, returns a short human-readable
   phrase explaining why (e.g. "its DeformableModel has 1 registered deformable
   body"), for MultibodyPlant to embed in its "(because ...)" diagnostic.
   Returns "" when this component does not block conversion to `scalar`. A
   non-empty phrase MUST be consistent with the matching is_cloneable_to_*()
   bool (non-empty implies that bool returns false); the reason is never used to
   decide convertibility. */
  virtual std::string GetScalarConversionFailureReason(
      const std::type_info&) const {
    return {};
  }
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::ScalarConvertibleComponent);
