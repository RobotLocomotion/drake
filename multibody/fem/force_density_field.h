#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

template <typename T>
class ForceDensityFieldImpl;

/** (Advanced) Enum for the type of force density in ForceDensityField. */
enum class ForceDensityType {
  /** ForceDensityField::EvaluateAt() returns the force per unit of _current_
   (deformed) configuration volume. */
  kPerCurrentVolume,
  /** ForceDensityField::EvaluateAt() returns the force per unit of _reference_
   configuration volume where the reference undeformed configuration is defined
   by the input mesh provided by the user. */
  kPerReferenceVolume,
};

/** The %ForceDensityField class is an abstract base class that represents a
 force density field affecting deformable bodies in a MultibodyPlant. The
 force field is described by the member function EvaluateAt() which takes as
 input a position in the world frame and returns the force density from the
 force density field at the given location, with unit [N/m³]. To create a
 concrete %ForceDensityField class, inherit from ForceDensityFieldImpl instead
 of directly inheriting from %ForceDensityField.
 @tparam_default_scalar */
template <typename T>
class ForceDensityField {
 public:
  virtual ~ForceDensityField() = 0;

  /** Evaluates the force density [N/m³] with the given `context` of the
   owning MultibodyPlant and a position in world, `p_WQ`. */
  Vector3<T> EvaluateAt(const systems::Context<T>& context,
                        const Vector3<T>& p_WQ) const {
    return DoEvaluateAt(context, p_WQ);
  }

  /** Returns an identical copy of `this` ForceDensityField. */
  std::unique_ptr<ForceDensityField<T>> Clone() const { return DoClone(); }

  /* (Advanced) Returns the force density type of `this` %ForceDensityFieldImpl.
   */
  ForceDensityType density_type() const { return density_type_; }

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ForceDensityField);

  /** Derived classes must override this function to provide a threadsafe
   implemention to the NVI EvaluateAt(). */
  virtual Vector3<T> DoEvaluateAt(const systems::Context<T>& context,
                                  const Vector3<T>& p_WQ) const = 0;

  /** Derived classes must override this function to implement the NVI
   Clone(). */
  virtual std::unique_ptr<ForceDensityField<T>> DoClone() const = 0;

 private:
  friend class ForceDensityFieldImpl<T>;

  /* Private constructor exposed only to ForceDensityFieldImpl. This prevents
   users from creating concrete force densities that directly inherit from
   ForceDensityField. */
  explicit ForceDensityField(ForceDensityType density_type)
      : density_type_(density_type) {}

  ForceDensityType density_type_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::ForceDensityField);
