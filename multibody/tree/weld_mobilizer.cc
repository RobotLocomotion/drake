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
void WeldMobilizer<T>::DoCalcNDotMatrix(const systems::Context<T>&,
                                        EigenPtr<MatrixX<T>>) const {}

template <typename T>
void WeldMobilizer<T>::DoCalcNplusDotMatrix(const systems::Context<T>&,
                                            EigenPtr<MatrixX<T>>) const {}

template <typename T>
void WeldMobilizer<T>::DoMapVelocityToQDot(const systems::Context<T>&,
                                           const Eigen::Ref<const VectorX<T>>&,
                                           EigenPtr<VectorX<T>>) const {}

template <typename T>
void WeldMobilizer<T>::DoMapQDotToVelocity(const systems::Context<T>&,
                                           const Eigen::Ref<const VectorX<T>>&,
                                           EigenPtr<VectorX<T>>) const {}

template <typename T>
void WeldMobilizer<T>::DoMapAccelerationToQDDot(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>&,
    EigenPtr<VectorX<T>>) const {}

template <typename T>
void WeldMobilizer<T>::DoMapQDDotToAcceleration(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>&,
    EigenPtr<VectorX<T>>) const {}

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
