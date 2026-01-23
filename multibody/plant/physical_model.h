#pragma once

#include <memory>
#include <set>
#include <string>
#include <variant>

#include "drake/common/default_scalars.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/scalar_convertible_component.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {

template <typename T>
class MultibodyPlant;

/* Forward declarations of all concrete PhysicalModel (right now one). */
template <typename T>
class DeformableModel;

/* Variant over const pointers to all PhysicalModel.
 MultibodyPlant owns all the physical models, as PhysicalModel pointers. Now,
 discrete update manager must create concrete instances for each physical model.
 They do so by using the visitor pattern. Therefore,
 a variant is used here so that discrete update managers can use the visitor
 pattern to create concrete physical models, solely from a pointer to the base
 class PhysicalModel. */
// N.B. For testing, we allow std::monostate to indicate an "empty model" in the
// return from PhysicalModel::DoToPhysicalModelPointerVariant().
template <typename T>
using PhysicalModelPointerVariant =
    std::variant<const DeformableModel<T>*, std::monostate>;

// TODO(xuchenhan-tri): Move PhysicalModel into internal namespace.
/** (Internal) PhysicalModel provides the functionalities to extend the type of
 physical model of MultibodyPlant. Developers can derive from this
 PhysicalModel to incorporate additional model elements coupled with the
 rigid body dynamics. For instance, simulation of deformable objects requires
 additional state and ports to interact with externals systems such as
 visualization.

 Similar to the routine of adding multiple model elements in MultibodyPlant,
 users should add all the model elements they wish to add to a PhysicalModel
 before the owning MultibodyPlant calls `Finalize()`. When `Finalize()` is
 invoked, MultibodyPlant will allocate the system level context resources for
 each PhysicalModel it owns. After the system resources are allocated, model
 mutation in the PhysicalModels owned by MultibodyPlant is not allowed.

 This class is for internal use only. Use derived concrete models (e.g.
 DeformableModel) instead.

 @tparam_default_scalar */
template <typename T>
class PhysicalModel : public internal::ScalarConvertibleComponent<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PhysicalModel);

  // TODO(xuchenhan-tri): We should also check that `owning_plant` is not
  // finalized yet. We can't do that just yet because
  // MultibodyPlant::is_finalized() currently lies and report true even when
  // Finalize() is not yet called (e.g. during scalar conversion).
  /** Constructs a PhysicalModel owned by the given `owning_plant`. The lifespan
   of the `owning_plant` must outlast `this` PhysicalModel. This PhysicalModel
   declares System resources from the `owning_plant` in the call to
   `DeclareSystemResources()` through the call to `MultibodyPlant::Finalize()`.
   @pre owning_plant != nullptr. */
  explicit PhysicalModel(MultibodyPlant<T>* owning_plant);

  ~PhysicalModel() override;

  /** (Internal only) Creates a clone of `this` concrete PhysicalModel object
   with the scalar type `ScalarType` to be owned by the given `plant`. The clone
   should be a deep copy of the original PhysicalModel with the exception of
   members overwritten in `DeclareSystemResources()`. This method is meant to be
   called by the scalar-converting copy constructor of MultibodyPlant only and
   thus is only called from a finalized MultibodyPlant.
   @tparam_default_scalar
   @throw std::exception if plant is nullptr.
   @throw std::exception if the owning plant of `this` PhysicalModel is
   not finalized.
   @param[in] plant pointer to the MultibodyPlant owning the clone. This needs
   to be a mutable pointer because the constructor of the clone requires a
   mutable pointer to the owning plant.
   @note `DeclareSystemResources()` is not called on the clone and needs to be
   called from the plant owning the clone. */
  template <typename ScalarType>
  std::unique_ptr<PhysicalModel<ScalarType>> CloneToScalar(
      MultibodyPlant<ScalarType>* plant) const {
    DRAKE_THROW_UNLESS(plant != nullptr);
    /* The plant owning `this` PhysicalModel must be finalized. */
    if (!this->plant().is_finalized()) {
      throw std::logic_error(
          "The owning plant of the PhysicalModel to be cloned must be "
          "finalized.");
    }
    if constexpr (std::is_same_v<ScalarType, double>) {
      return CloneToDouble(plant);
    } else if constexpr (std::is_same_v<ScalarType, AutoDiffXd>) {
      return CloneToAutoDiffXd(plant);
    } else if constexpr (std::is_same_v<ScalarType, symbolic::Expression>) {
      return CloneToSymbolic(plant);
    }
    DRAKE_UNREACHABLE();
  }

  /** Defaults to false. Derived classes that support making a clone that uses
   double as a scalar type must override this to return true. */
  bool is_cloneable_to_double() const override;

  /** Defaults to false. Derived classes that support making a clone that uses
   AutoDiffXd as a scalar type must override this to return true. */
  bool is_cloneable_to_autodiff() const override;

  /** Defaults to false. Derived classes that support making a clone that uses
   symbolic::Expression as a scalar type must override this to return true. */
  bool is_cloneable_to_symbolic() const override;

  /** (Internal only) MultibodyPlant calls this from within Finalize() to
   declare additional system resources. This method is only meant to be called
   by MultibodyPlant. The pointer to the owning plant is nulled after call to
   this function. */
  void DeclareSystemResources();

  /** (Internal only) Declares zero or more output ports in the owning
   MultibodyPlant to communicate with a SceneGraph.
   @throws std::exception if called after call to DeclareSystemResources().
   @throws std::exception if called more than once when at least one output port
   is created. */
  void DeclareSceneGraphPorts();

  /** Returns (a const pointer to) the specific model variant of `this`
   PhysicalModel. Note that the variant contains a pointer to the concrete model
   and therefore should not persist longer than the lifespan of this model.  */
  PhysicalModelPointerVariant<T> ToPhysicalModelPointerVariant() const {
    return DoToPhysicalModelPointerVariant();
  }

  /* Returns MultibodyPlant owning `this` PhysicalModel. */
  const MultibodyPlant<T>& plant() const { return owning_plant_; }

 protected:
  /* Returns the mutable back pointer to the MultibodyPlant owning `this`
   PhysicalModel pre-finalize and nullptr post-finalize. */
  MultibodyPlant<T>* mutable_plant() { return mutable_owning_plant_; }

  /* Returns the MultibodyTree associated with the MultibodyPlant that owns this
   physical model. */
  const internal::MultibodyTree<T>& internal_tree() const;

  /* Derived classes must override this function to return their specific model
   variant. */
  virtual PhysicalModelPointerVariant<T> DoToPhysicalModelPointerVariant()
      const = 0;

  /* Derived classes that support making a clone that uses double as a scalar
   type must implement this so that it creates a copy of the object with double
   as the scalar type. It should copy all members except for those overwritten
   in `DeclareSystemResources()`.
   @pre is_cloneable_to_double() == true. */
  virtual std::unique_ptr<PhysicalModel<double>> CloneToDouble(
      MultibodyPlant<double>* plant) const;

  /* Derived classes that support making a clone that uses AutoDiffXd as a
   scalar type must implement this so that it creates a copy of the object with
   AutoDiffXd as the scalar type. It should copy all members except for those
   overwritten in `DeclareSystemResources()`.
   @pre is_cloneable_to_autodiff() == true. */
  virtual std::unique_ptr<PhysicalModel<AutoDiffXd>> CloneToAutoDiffXd(
      MultibodyPlant<AutoDiffXd>* plant) const;

  /* Derived classes that support making a clone that uses symbolic::Expression
   as a scalar type must implement this so that it creates a copy of the object
   with symbolic::Expression as the scalar type. It should copy all members
   except for those overwritten in `DeclareSystemResources()`.
   @pre is_cloneable_to_symbolic() == true. */
  virtual std::unique_ptr<PhysicalModel<symbolic::Expression>> CloneToSymbolic(
      MultibodyPlant<symbolic::Expression>* plant) const;

  /* Derived class must override this to declare system resources for its
   specific model. */
  virtual void DoDeclareSystemResources() = 0;

  /* Derived class may choose to override this to declare ports in the owning
   MultibodyPlant to communicate with SceneGraph. */
  virtual void DoDeclareSceneGraphPorts() {}

  /* Helper method for throwing an exception within public methods that should
   not be called after system resources are declared. The invoking method should
   pass its name so that the error message can include that detail. */
  void ThrowIfSystemResourcesDeclared(const char* function_name) const;

  /* Helper method for throwing an exception within public methods that should
   not be called before system resources are declared. The invoking method
   should pass its name so that the error message can include that detail. */
  void ThrowIfSystemResourcesNotDeclared(const char* function_name) const;

  /* Returns the SceneGraph with which the given `plant` has been registered.
   @pre Finalize() has not been called on `plant`.
   @pre `plant` has been registered with some SceneGraph. */
  geometry::SceneGraph<T>& mutable_scene_graph();

  /* Protected LeafSystem methods exposed through MultibodyPlant.
   @throws std::exception if called after DeclareSystemResources() has finished.
  */
  systems::DiscreteStateIndex DeclareDiscreteState(
      const VectorX<T>& model_value);

  systems::AbstractParameterIndex DeclareAbstractParameter(
      const AbstractValue& model_value);

  systems::LeafOutputPort<T>& DeclareAbstractOutputPort(
      std::string name,
      typename systems::LeafOutputPort<T>::AllocCallback alloc_function,
      typename systems::LeafOutputPort<T>::CalcCallback calc_function,
      std::set<systems::DependencyTicket> prerequisites_of_calc = {
          systems::System<T>::all_sources_ticket()});

  systems::LeafOutputPort<T>& DeclareVectorOutputPort(
      std::string name, const systems::BasicVector<T>& model_vector,
      typename systems::LeafOutputPort<T>::CalcVectorCallback
          vector_calc_function,
      std::set<systems::DependencyTicket> prerequisites_of_calc = {
          systems::System<T>::all_sources_ticket()});

 private:
  /* Back pointer to the MultibodyPlant owning `this` PhysicalModel. */
  const MultibodyPlant<T>& owning_plant_;
  /* Mutable back pointer to the MultibodyPlant owning `this` PhysicalModel.
   Nullified post-finalize. */
  MultibodyPlant<T>* mutable_owning_plant_{nullptr};
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::PhysicalModel);
