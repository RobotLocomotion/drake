#include "drake/multibody/tree/mobilized_body_impl.h"

#include <memory>

#include "drake/multibody/tree/body_node_impl.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T, int nq, int nv>
std::unique_ptr<internal::BodyNode<T>>
MobilizedBodyImpl<T, nq, nv>::CreateBodyNode(
    const internal::BodyNode<T>* parent_node, const Body<T>* body,
    const MobilizedBody<T>* mobilizer) const {
  return std::make_unique<internal::BodyNodeImpl<T, nq, nv>>(parent_node, body,
                                                             mobilizer);
}

// Helper classes to aid the explicit instantiation with macro
// DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(), which
// expects a template class as an argument.
template <typename T>
class MobilizerImpl0x0 : public MobilizedBodyImpl<T, 0, 0> {};
template <typename T>
class MobilizerImpl1x1 : public MobilizedBodyImpl<T, 1, 1> {};
template <typename T>
class MobilizerImpl2x2 : public MobilizedBodyImpl<T, 2, 2> {};
template <typename T>
class MobilizerImpl3x3 : public MobilizedBodyImpl<T, 3, 3> {};
template <typename T>
class MobilizerImpl6x6 : public MobilizedBodyImpl<T, 6, 6> {};
template <typename T>
class MobilizerImpl7x6 : public MobilizedBodyImpl<T, 7, 6> {};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::MobilizerImpl0x0)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::MobilizerImpl1x1)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::MobilizerImpl2x2)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::MobilizerImpl3x3)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::MobilizerImpl6x6)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::MobilizerImpl7x6)
