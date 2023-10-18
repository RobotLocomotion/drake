#pragma once

#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/multibody_element.h"

namespace drake {
namespace multibody {

template <typename T>
class ExternalForceField : public MultibodyElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ExternalForceField)

  /** Evaluates the force density field at the given position in world, `p_WQ`.
   */
  Vector3<T> EvaluateAt(const systems::Context<T>& context,
                        const Vector3<T>& p_WQ) const {
    return DoEvaluateAt(context, p_WQ);
  }

  /** Declares MultibodyTreeSystem cache entries at
   MultibodyTreeSystem::Finalize() time. NVI to the virtual method
   DoDeclareCacheEntries().
   @param[in] tree_system A mutable copy of the parent MultibodyTreeSystem.
   @pre 'tree_system' must be the same as the parent tree system (what's
   returned from GetParentTreeSystem()). */
  void DeclareCacheEntries(internal::MultibodyTreeSystem<T>* tree_system) {
    tree_system_ = tree_system;
    DoDeclareCacheEntries(tree_system);
  }

 protected:
  ExternalForceField() = default;

  virtual Vector3<T> DoEvaluateAt(const systems::Context<T>& context,
                                  const Vector3<T>& p_WQ) const = 0;

  const internal::MultibodyTreeSystem<T>& tree_system() const {
    return *tree_system_;
  }

  /** Implementation of the NVI DeclareCacheEntries(). MultibodyElement-derived
   objects may override to declare their specific cache entries. */
  virtual void DoDeclareCacheEntries(internal::MultibodyTreeSystem<T>*) {}

 private:
  void DoSetTopology(const internal::MultibodyTreeTopology&) final{};

  const internal::MultibodyTreeSystem<T>* tree_system_{nullptr};
};

template <typename T>
class ExplicitExternalForceField : public ExternalForceField<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ExplicitExternalForceField)

  /* Constructs an explicit external force field that reports that value of an
   std::function. */
  ExplicitExternalForceField(std::function<Vector3<T>(const Vector3<T>&)> field)
      : field_(std::move(field)) {}

 private:
  Vector3<T> DoEvaluateAt(const systems::Context<T>&,
                          const Vector3<T>& p_WQ) const final {
    return field_(p_WQ);
  };

 private:
  std::function<Vector3<T>(const Vector3<T>&)> field_;
};

template <typename T>
class GravityForceField : public ExternalForceField<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GravityForceField)

  GravityForceField(const Vector3<T>& gravity_vector, const T& density)
      : force_density_(density * gravity_vector) {}

 private:
  Vector3<T> DoEvaluateAt(const systems::Context<T>&,
                          const Vector3<T>&) const final {
    return force_density_;
  };

 private:
  Vector3<T> force_density_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::ExternalForceField)
