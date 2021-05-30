#include "drake/multibody/tree/screw_mobilizer.h"

namespace drake {
namespace multibody {
namespace internal {


template <typename T>
const T ScrewMobilizer<T>::get_translation(
    const systems::Context<T>& context) const {
  auto q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);
  return q.coeffRef(0);
}

template <typename T>
const ScrewMobilizer<T>& ScrewMobilizer<T>::set_translation(
    systems::Context<T>* context,
    const T& translation) const {
  auto q = this->get_mutable_positions(&*context);
  DRAKE_ASSERT(q.size() == kNq);
  q[0] = translation;
  return *this;
}

template <typename T>
const T& ScrewMobilizer<T>::get_angle(
    const systems::Context<T>& context) const {
  auto q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);
  return q.coeffRef(1);
}

template <typename T>
const ScrewMobilizer<T>& ScrewMobilizer<T>::set_angle(
    systems::Context<T>* context, const T& angle) const {
  auto q = this->get_mutable_positions(&*context);
  DRAKE_ASSERT(q.size() == kNq);
  q[1] = angle;
  return *this;
}

template <typename T>
const T ScrewMobilizer<T>::get_translation_rate(
    const systems::Context<T>& context) const {
  const auto& v = this->get_velocities(context);
  DRAKE_ASSERT(v.size() == kNv);
  return v.coeffRef(0);
}

template <typename T>
const ScrewMobilizer<T>& ScrewMobilizer<T>::set_translation_rate(
    systems::Context<T>* context,
    const T& v_FM_F) const {
  auto v = this->get_mutable_velocities(&*context);
  DRAKE_ASSERT(v.size() == kNv);
  v[0] = v_FM_F;
  DRAKE_ASSERT(false);
  return *this;
}

template <typename T>
const T& ScrewMobilizer<T>::get_angular_rate(
    const systems::Context<T>& context) const {
  const auto& v = this->get_velocities(context);
  DRAKE_ASSERT(v.size() == kNv);
  return v.coeffRef(1);
}

template <typename T>
const ScrewMobilizer<T>& ScrewMobilizer<T>::set_angular_rate(
    systems::Context<T>* context, const T& theta_dot) const {
  auto v = this->get_mutable_velocities(&*context);
  DRAKE_ASSERT(v.size() == kNv);
  v[1] = theta_dot;
  return *this;
}

template <typename T>
math::RigidTransform<T> ScrewMobilizer<T>::CalcAcrossMobilizerTransform(
    const systems::Context<T>& context) const {
  const auto& q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);
  Vector3<T> X_FM_translation;
  X_FM_translation << 0.0, 0.0, q[0];
  return math::RigidTransform<T>(math::RotationMatrix<T>::MakeZRotation(q[1]),
                                 X_FM_translation);
}

template <typename T>
SpatialVelocity<T> ScrewMobilizer<T>::CalcAcrossMobilizerSpatialVelocity(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& v) const {
  DRAKE_ASSERT(v.size() == kNv);
  Vector6<T> V_FM_vector;
  V_FM_vector << 0.0, 0.0, v[1], 0.0, 0.0, v[0];
  return SpatialVelocity<T>(V_FM_vector);
}

template <typename T>
SpatialAcceleration<T>
ScrewMobilizer<T>::CalcAcrossMobilizerSpatialAcceleration(
    const systems::Context<T>&,
    const Eigen::Ref<const VectorX<T>>& vdot) const {
  DRAKE_ASSERT(vdot.size() == kNv);
  Vector6<T> A_FM_vector;
  A_FM_vector << 0.0, 0.0, vdot[1], 0.0, 0.0, vdot[0];
  return SpatialAcceleration<T>(A_FM_vector);
}

template <typename T>
void ScrewMobilizer<T>::ProjectSpatialForce(const systems::Context<T>&,
                                             const SpatialForce<T>& F_Mo_F,
                                             Eigen::Ref<VectorX<T>> tau) const {
  DRAKE_ASSERT(tau.size() == kNv);
  tau.head(2) = F_Mo_F.translational().head(2);
  tau[2] = F_Mo_F.rotational()[2];
}

template <typename T>
void ScrewMobilizer<T>::DoCalcNMatrix(const systems::Context<T>&,
                                       EigenPtr<MatrixX<T>> N) const {
  *N = Matrix2<T>::Identity();
}

template <typename T>
void ScrewMobilizer<T>::DoCalcNplusMatrix(const systems::Context<T>&,
                                           EigenPtr<MatrixX<T>> Nplus) const {
  *Nplus = Matrix2<T>::Identity();
}

template <typename T>
void ScrewMobilizer<T>::MapVelocityToQDot(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& v,
    EigenPtr<VectorX<T>> qdot) const {
  DRAKE_ASSERT(v.size() == kNv);
  DRAKE_ASSERT(qdot != nullptr);
  DRAKE_ASSERT(qdot->size() == kNq);
  *qdot = v;
}

template <typename T>
void ScrewMobilizer<T>::MapQDotToVelocity(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& qdot,
    EigenPtr<VectorX<T>> v) const {
  DRAKE_ASSERT(qdot.size() == kNq);
  DRAKE_ASSERT(v != nullptr);
  DRAKE_ASSERT(v->size() == kNv);
  *v = qdot;
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Mobilizer<ToScalar>>
ScrewMobilizer<T>::TemplatedDoCloneToScalar(
    const MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& inboard_frame_clone =
      tree_clone.get_variant(this->inboard_frame());
  const Frame<ToScalar>& outboard_frame_clone =
      tree_clone.get_variant(this->outboard_frame());
  return std::make_unique<ScrewMobilizer<ToScalar>>(inboard_frame_clone,
                                                     outboard_frame_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<double>> ScrewMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<AutoDiffXd>> ScrewMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<symbolic::Expression>>
ScrewMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::ScrewMobilizer)
