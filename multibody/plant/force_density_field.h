#pragma once

#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/multibody_tree_system.h"

namespace drake {
namespace multibody {

template <typename T>
class MultibodyPlant;

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
 force density field at the given location, with unit [N/m³]. The force density
 function can depend on other Context-dependent parameters.
 @tparam_default_scalar */
template <typename T>
class ForceDensityField {
 public:
  virtual ~ForceDensityField();

  /** Evaluates the force density [N/m³] with the given `context` of the
   owning MultibodyPlant and a position in world, `p_WQ`. */
  Vector3<T> EvaluateAt(const systems::Context<T>& context,
                        const Vector3<T>& p_WQ) const {
    return DoEvaluateAt(context, p_WQ);
  }

  /** Returns an identical copy of `this` ForceDensityField. */
  std::unique_ptr<ForceDensityField<T>> Clone() const { return DoClone(); }

  /** Returns true iff `this` external force is owned by a MultibodyPlant. */
  bool has_parent_system() const { return tree_system_ != nullptr; }

  /** Returns the owning %MultibodyPlant %LeafSystem.
   @throws std::exception if `this` force density field is not owned by any
   system. */
  const systems::LeafSystem<T>& parent_system_or_throw() const {
    DRAKE_THROW_UNLESS(tree_system_ != nullptr);
    return *tree_system_;
  }

  /* (Advanced) Returns the force density type of `this` %ForceDensityField. */
  ForceDensityType density_type() const { return density_type_; }

#ifndef DRAKE_DOXYGEN_CXX
  /* (Internal use only) Declares internal::MultibodyTreeSystem cache
   entries/parameters/input ports at Finalize() time. This is useful if the
   external force is context-dependent. For example, if the force density field
   can be turned on/off depending on an external input from another System, the
   particular force density field subclass can open an input port to the owning
   MultibodyTreeSystem to receive the signal. This class holds onto a pointer
   to the given `tree_system` and thus `tree_system` must outlive `this` object.
   @param[in] tree_system A mutable pointer to the owning system.
   @throws std::exception if the `tree_system` does not belong to a
                          MultibodyPlant.
   @note You can only invoke this function if you have a definition of
         MultibodyPlant available. That is, you must include
        `drake/multibody/plant/multibody_plant.h` in the translation unit that
         invokes this function; force_density_field.h cannot do that for you.
   @pre tree_system != nullptr. */
  template <typename MultibodyPlantDeferred = MultibodyPlant<T>>
  void DeclareSystemResources(internal::MultibodyTreeSystem<T>* tree_system);
  // N.B. The implementation of DeclareSystemResources is provided as part of
  // the plant library in force_density_field_declare_system_resources.cc, to
  // avoid a dependency cycle.
#endif

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ForceDensityField);

  explicit ForceDensityField(
      ForceDensityType density_type = ForceDensityType::kPerCurrentVolume)
      : density_type_(density_type) {}

  /** Derived classes must override this function to provide a threadsafe
   implemention to the NVI EvaluateAt(). */
  virtual Vector3<T> DoEvaluateAt(const systems::Context<T>& context,
                                  const Vector3<T>& p_WQ) const = 0;

  /** Derived classes must override this function to implement the NVI
   Clone(). */
  virtual std::unique_ptr<ForceDensityField<T>> DoClone() const = 0;

  /** NVI implementations for declaring system resources. Defaults to no-op.
   Derived classes should override the default implementation if the external
   force field is Context-dependent.
   @{ */
  virtual void DoDeclareCacheEntries(MultibodyPlant<T>*) {}
  virtual void DoDeclareInputPorts(MultibodyPlant<T>*) {}
  /** }@ */

  /** Protected LeafSystem methods exposed to declare system resources in a
   MultibodyPlant. DoDeclareCacheEntries() and DoDeclareInputPorts() can use
   these to declare cache entries and input ports. */
  static systems::CacheEntry& DeclareCacheEntry(
      internal::MultibodyTreeSystem<T>* plant, std::string description,
      systems::ValueProducer value_producer,
      std::set<systems::DependencyTicket> prerequisites_of_calc);
  static systems::InputPort<T>& DeclareAbstractInputPort(
      internal::MultibodyTreeSystem<T>* plant, std::string name,
      const AbstractValue& model_value);
  static systems::InputPort<T>& DeclareVectorInputPort(
      internal::MultibodyTreeSystem<T>* plant, std::string name,
      const systems::BasicVector<T>& model_vector);

 private:
  /* Declares MultibodyPlant cache entries at Finalize() time. NVI to the
   virtual method DoDeclareCacheEntries().
   @param[in] tree_system A mutable pointer to the owning system. */
  void DeclareCacheEntries(MultibodyPlant<T>* plant) {
    DRAKE_DEMAND(plant != nullptr);
    DoDeclareCacheEntries(plant);
  }

  /* Declares MultibodyPlant input ports at Finalize() time. NVI to the
   virtual method DoDeclareInputPorts().
   @param[in] tree_system A mutable pointer to the owning system. */
  void DeclareInputPorts(MultibodyPlant<T>* plant) {
    DRAKE_DEMAND(plant != nullptr);
    DoDeclareInputPorts(plant);
  }

  const internal::MultibodyTreeSystem<T>* tree_system_{nullptr};
  ForceDensityType density_type_{ForceDensityType::kPerCurrentVolume};
};

/** A uniform gravitational force density field for a uniform density object.
 The force density f [N/m³] is given by the product of mass density
 ρ [kg/m³] and gravity vector g [m/s²].
 @tparam_default_scalar */
template <typename T>
class GravityForceField : public ForceDensityField<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GravityForceField);

  /** Constructs a uniform gravitational force density field for a uniform
  density object with the given `gravity_vector` [m/s²] and `mass_density`
  [kg/m³] in the reference (undeformed) configuration where the reference
  (undeformed) configuration is defined by the input mesh provided by the user.
  */
  GravityForceField(const Vector3<T>& gravity_vector, const T& mass_density)
      : ForceDensityField<T>(ForceDensityType::kPerReferenceVolume),
        force_density_(mass_density * gravity_vector) {}

  ~GravityForceField() override;

 private:
  Vector3<T> DoEvaluateAt(const systems::Context<T>&,
                          const Vector3<T>&) const final {
    return force_density_;
  };

  std::unique_ptr<ForceDensityField<T>> DoClone() const final {
    return std::make_unique<GravityForceField<T>>(*this);
  }

  /* The constant force density at all points in world. */
  Vector3<T> force_density_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::ForceDensityField);

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::GravityForceField);
