#pragma once

#include <memory>
#include <set>
#include <string>
#include <variant>

#include "drake/common/default_scalars.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/scalar_convertible_component.h"
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

  PhysicalModel() = default;

  ~PhysicalModel() override = default;

  /** Creates a clone of the concrete PhysicalModel object with the scalar type
   `ScalarType`. The clone should be a deep copy of the original PhysicalModel
   with the exception of members overwritten in `DeclareSystemResources()`. This
   method is meant to be called by the scalar-converting copy constructor of
   MultibodyPlant only.
   @tparam_default_scalar */
  template <typename ScalarType>
  std::unique_ptr<PhysicalModel<ScalarType>> CloneToScalar() const {
    if constexpr (std::is_same_v<ScalarType, double>) {
      return CloneToDouble();
    } else if constexpr (std::is_same_v<ScalarType, AutoDiffXd>) {
      return CloneToAutoDiffXd();
    } else if constexpr (std::is_same_v<ScalarType, symbolic::Expression>) {
      return CloneToSymbolic();
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

  /** MultibodyPlant calls this from within Finalize() to declare additional
   system resources. This method is only meant to be called by MultibodyPlant.
   We pass in a MultibodyPlant pointer so that derived PhysicalModels can use
   specific MultibodyPlant cache tickets.
   @pre plant != nullptr. */
  void DeclareSystemResources(MultibodyPlant<T>* plant) {
    DRAKE_DEMAND(plant != nullptr);
    DoDeclareSystemResources(plant);
    system_resources_declared_ = true;
  }

  /** Returns (a const pointer to) the specific model variant of `this`
   PhysicalModel. Note that the variant contains a pointer to the concrete model
   and therefore should not persist longer than the lifespan of this model.  */
  PhysicalModelPointerVariant<T> ToPhysicalModelPointerVariant() const {
    return DoToPhysicalModelPointerVariant();
  }

 protected:
  /* Derived classes must override this function to return their specific model
   variant. */
  virtual PhysicalModelPointerVariant<T> DoToPhysicalModelPointerVariant()
      const = 0;

  /* Derived classes that support making a clone that uses double as a scalar
   type must implement this so that it creates a copy of the object with double
   as the scalar type. It should copy all members except for those overwritten
   in `DeclareSystemResources()`. */
  virtual std::unique_ptr<PhysicalModel<double>> CloneToDouble() const;

  /* Derived classes that support making a clone that uses AutoDiffXd as a
   scalar type must implement this so that it creates a copy of the object with
   AutoDiffXd as the scalar type. It should copy all members except for those
   overwritten in `DeclareSystemResources()`. */
  virtual std::unique_ptr<PhysicalModel<AutoDiffXd>> CloneToAutoDiffXd() const;

  /* Derived classes that support making a clone that uses symbolic::Expression
   as a scalar type must implement this so that it creates a copy of the object
   with symbolic::Expression as the scalar type. It should copy all members
   except for those overwritten in `DeclareSystemResources()`. */
  virtual std::unique_ptr<PhysicalModel<symbolic::Expression>> CloneToSymbolic()
      const;

  /* Derived class must override this to declare system resources for its
   specific model. */
  virtual void DoDeclareSystemResources(MultibodyPlant<T>* plant) = 0;

  /* Helper method for throwing an exception within public methods that should
   not be called after system resources are declared. The invoking method should
   pass its name so that the error message can include that detail. */
  void ThrowIfSystemResourcesDeclared(const char* source_method) const {
    if (system_resources_declared_) {
      throw std::logic_error(
          "Calls to '" + std::string(source_method) +
          "()' after system resources have been declared are not allowed.");
    }
  }

  /* Helper method for throwing an exception within public methods that should
   not be called before system resources are declared. The invoking method
   should pass its name so that the error message can include that detail. */
  void ThrowIfSystemResourcesNotDeclared(const char* source_method) const {
    if (!system_resources_declared_) {
      throw std::logic_error(
          "Calls to '" + std::string(source_method) +
          "()' before system resources have been declared are not allowed.");
    }
  }

  /* Returns the SceneGraph with which the given `plant` has been registered.
   @pre plant != nullptr.
   @pre Finalize() has not been called on `plant`.
   @pre `plant` has been registered with some SceneGraph. */
  geometry::SceneGraph<T>& mutable_scene_graph(MultibodyPlant<T>* plant);

  /* Protected LeafSystem methods exposed through MultibodyPlant. */
  static systems::DiscreteStateIndex DeclareDiscreteState(
      MultibodyPlant<T>* plant, const VectorX<T>& model_value);

  static systems::LeafOutputPort<T>& DeclareAbstractOutputPort(
      MultibodyPlant<T>* plant, std::string name,
      typename systems::LeafOutputPort<T>::AllocCallback alloc_function,
      typename systems::LeafOutputPort<T>::CalcCallback calc_function,
      std::set<systems::DependencyTicket> prerequisites_of_calc = {
          systems::System<T>::all_sources_ticket()});

  static systems::LeafOutputPort<T>& DeclareVectorOutputPort(
      MultibodyPlant<T>* plant, std::string name,
      const systems::BasicVector<T>& model_vector,
      typename systems::LeafOutputPort<T>::CalcVectorCallback
          vector_calc_function,
      std::set<systems::DependencyTicket> prerequisites_of_calc = {
          systems::System<T>::all_sources_ticket()});

 private:
  /* Flag to track whether the system resources requested by `this`
   PhysicalModel have been declared. */
  bool system_resources_declared_{false};
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::PhysicalModel);
