#include "drake/multibody/tree/body.h"

#include <memory>

#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
template <typename ToScalar>
std::unique_ptr<Frame<ToScalar>> BodyFrame<T>::TemplatedDoCloneToScalar(
    const internal::MultibodyTree<ToScalar>& tree_clone) const {
  const Body<ToScalar>& body_clone =
      tree_clone.get_body(this->body().index());
  // BodyFrame's constructor cannot be called from std::make_unique since it is
  // private and therefore we use "new".
  return std::unique_ptr<BodyFrame<ToScalar>>(
      new BodyFrame<ToScalar>(body_clone));
}

template <typename T>
std::unique_ptr<Frame<double>> BodyFrame<T>::DoCloneToScalar(
    const internal::MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Frame<AutoDiffXd>> BodyFrame<T>::DoCloneToScalar(
    const internal::MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Frame<symbolic::Expression>> BodyFrame<T>::DoCloneToScalar(
    const internal::MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
ScopedName Body<T>::scoped_name() const {
  return ScopedName(
      this->get_parent_tree().GetModelInstanceName(this->model_instance()),
      name_);
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::BodyFrame)

// Ideally, we'd be instantiating the entire class here, instead of just one
// member function. However, the MultibodyTree physical design is so contrary to
// GSG best practices that trying to do the entire class here doesn't work.
DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    &drake::multibody::Body<T>::scoped_name
))
