#pragma once

namespace drake {
namespace multibody {
namespace internal {

/* An interface class for external auxiliary MultibodyPlant components that
 provides methods to query whether a component is scalar convertable to a
 particular scalar type.
 @tparam_default_scalar */
template <typename T>
class MultibodyPlantExternalComponent {
 public:
  virtual ~MultibodyPlantExternalComponent() = default;

  /* Returns true if `this` multibody plant external component can be scalar
   converted to double. */
  virtual bool is_cloneable_to_double() const = 0;

  /* Returns true if `this` multibody plant external component can be scalar
   converted to AutoDiffXd. */
  virtual bool is_cloneable_to_autodiff() const = 0;

  /* Returns true if `this` multibody plant external component can be scalar
   converted to symbolic::Expression. */
  virtual bool is_cloneable_to_symbolic() const = 0;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
