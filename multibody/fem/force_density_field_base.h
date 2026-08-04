#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

template <typename T>
class ForceDensityField;

/** (Advanced) Enum for the type of force density in ForceDensityFieldBase. */
enum class ForceDensityType {
  /** ForceDensityFieldBase::EvaluateAt() returns the force per unit of
   _current_ (deformed) configuration volume. */
  kPerCurrentVolume,
  /** ForceDensityFieldBase::EvaluateAt() returns the force per unit of
   _reference_ configuration volume where the reference undeformed configuration
   is defined by the input mesh provided by the user. */
  kPerReferenceVolume,
};

/** The %ForceDensityFieldBase class is an abstract base class that represents a
 force density field affecting deformable bodies in a MultibodyPlant. The
 force field is described by the member function EvaluateAt() which takes as
 input a position in the world frame and returns the force density from the
 force density field at the given location, with unit [N/m³]. To create a
 concrete %ForceDensityFieldBase class, inherit from ForceDensityField instead
 of directly inheriting from %ForceDensityFieldBase.
 @tparam_default_scalar */
template <typename T>
class ForceDensityFieldBase {
 public:
  virtual ~ForceDensityFieldBase() = 0;

  /** Evaluates the force density [N/m³] with the given `context` of the
   owning MultibodyPlant and a position in world, `p_WQ`. */
  Vector3<T> EvaluateAt(const systems::Context<T>& context,
                        const Vector3<T>& p_WQ) const {
    return DoEvaluateAt(context, p_WQ);
  }

  /** Returns an identical copy of `this` ForceDensityFieldBase. */
  std::unique_ptr<ForceDensityFieldBase<T>> Clone() const { return DoClone(); }

  /* (Advanced) Returns the force density type of `this` %ForceDensityField.
   */
  ForceDensityType density_type() const { return density_type_; }

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ForceDensityFieldBase);

  /** Derived classes must override this function to provide a threadsafe
   implemention to the NVI EvaluateAt(). */
  virtual Vector3<T> DoEvaluateAt(const systems::Context<T>& context,
                                  const Vector3<T>& p_WQ) const = 0;

  /** Derived classes must override this function to implement the NVI
   Clone(). */
  virtual std::unique_ptr<ForceDensityFieldBase<T>> DoClone() const = 0;

 private:
  friend class ForceDensityField<T>;

  /* Private constructor exposed only to ForceDensityField. This prevents
   users from creating concrete force densities that directly inherit from
   ForceDensityFieldBase. */
  explicit ForceDensityFieldBase(ForceDensityType density_type)
      : density_type_(density_type) {}

  ForceDensityType density_type_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::ForceDensityFieldBase);
