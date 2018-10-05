#include "drake/multibody/multibody_tree/mobilizer_impl.h"

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/multibody/multibody_tree/body_node_impl.h"

namespace drake {
namespace multibody {

template <typename T, int  nq, int nv>
std::unique_ptr<internal::BodyNode<T>> MobilizerImpl<T, nq, nv>::CreateBodyNode(
    const internal::BodyNode<T>* parent_node,
    const Body<T>* body, const Mobilizer<T>* mobilizer) const {
  return std::make_unique<internal::BodyNodeImpl<T, nq, nv>>(parent_node,
                                                             body, mobilizer);
}

// Helper classes to aid the explicit instantiation with macro
// DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(), which
// expects a template class as an argument.
template <typename T>
class MobilizerImpl0x0 : public MobilizerImpl<T, 0, 0> {};
template <typename T>
class MobilizerImpl1x1 : public MobilizerImpl<T, 1, 1> {};
template <typename T>
class MobilizerImpl2x2 : public MobilizerImpl<T, 2, 2> {};
template <typename T>
class MobilizerImpl3x3 : public MobilizerImpl<T, 3, 3> {};
template <typename T>
class MobilizerImpl7x6 : public MobilizerImpl<T, 7, 6> {};

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::MobilizerImpl0x0)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::MobilizerImpl1x1)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::MobilizerImpl2x2)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::MobilizerImpl3x3)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::MobilizerImpl7x6)
