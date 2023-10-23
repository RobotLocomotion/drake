#pragma once

#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/multibody_tree_system.h"

namespace drake {
namespace multibody {

/** External force field only affects deformable bodies. */
template <typename T>
class ExternalForceField {
 public:
  virtual ~ExternalForceField() = default;

  /** Evaluates the force density field at the given position in world, `p_WQ`.
   */
  Vector3<T> EvaluateAt(const Vector3<T>& p_WQ) const {
    return DoEvaluateAt(tree_system_context(), p_WQ);
  }

  virtual std::unique_ptr<ExternalForceField<T>> Clone() const {
    return DoClone();
  }

  /** (Internal use only) Declares internal::MultibodyTreeSystem cache entries
   at Finalize() time. NVI to the virtual method DoDeclareCacheEntries().
   @param[in] tree_system A mutable pointer to the owning system. */
  void DeclareCacheEntries(internal::MultibodyTreeSystem<T>* tree_system) {
    if (tree_system_ != nullptr) {
      DRAKE_DEMAND(tree_system_ == tree_system);
    } else {
      tree_system_ = tree_system;
    }
    DoDeclareCacheEntries(tree_system);
  }

  void DeclareInputPorts(internal::MultibodyTreeSystem<T>* tree_system) {
    if (tree_system_ != nullptr) {
      DRAKE_DEMAND(tree_system_ == tree_system);
    } else {
      tree_system_ = tree_system;
    }
    DoDeclareInputPorts(tree_system);
  }

  /** (Internal use only) Set the context to the owning system's context. */
  void set_tree_system_context(const systems::Context<T>* context) {
    tree_system_context_ = context;
  }

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ExternalForceField)

  ExternalForceField() = default;

  virtual Vector3<T> DoEvaluateAt(const systems::Context<T>& context,
                                  const Vector3<T>& p_WQ) const = 0;
  virtual std::unique_ptr<ExternalForceField<T>> DoClone() const = 0;
  virtual void DoDeclareCacheEntries(internal::MultibodyTreeSystem<T>*) {}
  virtual void DoDeclareInputPorts(internal::MultibodyTreeSystem<T>*) {}

  const internal::MultibodyTreeSystem<T>& tree_system() const {
    DRAKE_DEMAND(tree_system_ != nullptr);
    return *tree_system_;
  }

  const systems::Context<T>& tree_system_context() const {
    DRAKE_DEMAND(tree_system_context_ != nullptr);
    return *tree_system_context_;
  }

  static systems::CacheEntry& DeclareCacheEntry(
      internal::MultibodyTreeSystem<T>* tree_system, std::string description,
      systems::ValueProducer value_producer,
      std::set<systems::DependencyTicket> prerequisites_of_calc);

  static systems::InputPort<T>& DeclareAbstractInputPort(
      internal::MultibodyTreeSystem<T>* tree_system, std::string name,
      const AbstractValue& model_value);

  static systems::InputPort<T>& DeclareVectorInputPort(
      internal::MultibodyTreeSystem<T>* tree_system, std::string name,
      const systems::BasicVector<T>& model_vector);

 private:
  const internal::MultibodyTreeSystem<T>* tree_system_{nullptr};
  const systems::Context<T>* tree_system_context_{nullptr};
};

template <typename T>
class ExplicitExternalForceField final : public ExternalForceField<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ExplicitExternalForceField)

  /* Constructs an explicit external force field that reports that value of an
   std::function. */
  ExplicitExternalForceField(std::function<Vector3<T>(const Vector3<T>&)> field)
      : field_(std::move(field)) {}

 private:
  Vector3<T> DoEvaluateAt(const systems::Context<T>&,
                          const Vector3<T>& p_WQ) const final {
    return field_(p_WQ);
  };

  std::unique_ptr<ExternalForceField<T>> DoClone() const final {
    return std::make_unique<ExplicitExternalForceField<T>>(*this);
  }

  std::function<Vector3<T>(const Vector3<T>&)> field_;
};

template <typename T>
class GravityForceField : public ExternalForceField<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GravityForceField)

  GravityForceField(const Vector3<T>& gravity_vector, const T& density)
      : force_density_(density * gravity_vector) {}

 private:
  Vector3<T> DoEvaluateAt(const systems::Context<T>&,
                          const Vector3<T>&) const final {
    return force_density_;
  };

  std::unique_ptr<ExternalForceField<T>> DoClone() const final {
    return std::make_unique<GravityForceField<T>>(*this);
  }

  Vector3<T> force_density_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::ExternalForceField)
