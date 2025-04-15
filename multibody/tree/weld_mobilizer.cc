#include "drake/multibody/tree/weld_mobilizer.h"

#include <memory>

#include "drake/multibody/tree/body_node_impl.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
WeldMobilizer<T>::~WeldMobilizer() = default;

template <typename T>
std::unique_ptr<BodyNode<T>> WeldMobilizer<T>::CreateBodyNode(
    const BodyNode<T>* parent_node, const RigidBody<T>* body,
    const Mobilizer<T>* mobilizer) const {
  return std::make_unique<BodyNodeImpl<T, WeldMobilizer>>(parent_node, body,
                                                          mobilizer);
}

template <typename T>
math::RigidTransform<T> WeldMobilizer<T>::CalcAcrossMobilizerTransform(
    const systems::Context<T>&) const {
  return math::RigidTransform<T>();  // Identity
}

template <typename T>
SpatialVelocity<T> WeldMobilizer<T>::CalcAcrossMobilizerSpatialVelocity(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>&) const {
  return SpatialVelocity<T>::Zero();
}

template <typename T>
SpatialAcceleration<T> WeldMobilizer<T>::CalcAcrossMobilizerSpatialAcceleration(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>&) const {
  return SpatialAcceleration<T>::Zero();
}

template <typename T>
void WeldMobilizer<T>::ProjectSpatialForce(const systems::Context<T>&,
                                           const SpatialForce<T>&,
                                           Eigen::Ref<VectorX<T>> tau) const {
  DRAKE_ASSERT(tau.size() == kNv);
}

template <typename T>
void WeldMobilizer<T>::DoCalcNMatrix(const systems::Context<T>&,
                                     EigenPtr<MatrixX<T>>) const {}

template <typename T>
void WeldMobilizer<T>::DoCalcNplusMatrix(const systems::Context<T>&,
                                         EigenPtr<MatrixX<T>>) const {}

template <typename T>
void WeldMobilizer<T>::MapVelocityToQDot(const systems::Context<T>&,
                                         const Eigen::Ref<const VectorX<T>>& v,
                                         EigenPtr<VectorX<T>> qdot) const {
  DRAKE_ASSERT(v.size() == kNv);
  DRAKE_ASSERT(qdot != nullptr);
  DRAKE_ASSERT(qdot->size() == kNq);
}

template <typename T>
void WeldMobilizer<T>::MapQDotToVelocity(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& qdot,
    EigenPtr<VectorX<T>> v) const {
  DRAKE_ASSERT(qdot.size() == kNq);
  DRAKE_ASSERT(v != nullptr);
  DRAKE_ASSERT(v->size() == kNv);
}

template <typename T>
void WeldMobilizer<T>::MapAccelerationToQDDot(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& vdot,
    EigenPtr<VectorX<T>> qddot) const {
  DRAKE_ASSERT(vdot.size() == kNv);
  DRAKE_ASSERT(qddot != nullptr);
  DRAKE_ASSERT(qddot->size() == kNq);
}

template <typename T>
void WeldMobilizer<T>::MapQDDotToAcceleration(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& qddot,
    EigenPtr<VectorX<T>> vdot) const {
  DRAKE_ASSERT(qddot.size() == kNq);
  DRAKE_ASSERT(vdot != nullptr);
  DRAKE_ASSERT(vdot->size() == kNv);
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Mobilizer<ToScalar>> WeldMobilizer<T>::TemplatedDoCloneToScalar(
    const MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& inboard_frame_clone =
      tree_clone.get_variant(this->inboard_frame());
  const Frame<ToScalar>& outboard_frame_clone =
      tree_clone.get_variant(this->outboard_frame());
  return std::make_unique<WeldMobilizer<ToScalar>>(
      tree_clone.get_mobod(this->mobod().index()), inboard_frame_clone,
      outboard_frame_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<double>> WeldMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<AutoDiffXd>> WeldMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<symbolic::Expression>>
WeldMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::WeldMobilizer);
