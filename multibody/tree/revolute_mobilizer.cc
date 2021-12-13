#include "drake/multibody/tree/revolute_mobilizer.h"

#include <memory>
#include <stdexcept>

#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
std::string RevoluteMobilizer<T>::position_suffix(
  int position_index_in_mobilizer) const {
  if (position_index_in_mobilizer == 0) {
    return "q";
  }
  throw std::runtime_error("RevoluteMobilizer has only 1 position.");
}

template <typename T>
std::string RevoluteMobilizer<T>::velocity_suffix(
  int velocity_index_in_mobilizer) const {
  if (velocity_index_in_mobilizer == 0) {
    return "w";
  }
  throw std::runtime_error("RevoluteMobilizer has only 1 velocity.");
}

template <typename T>
const T& RevoluteMobilizer<T>::get_angle(
    const systems::Context<T>& context) const {
  auto q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);
  return q.coeffRef(0);
}

template <typename T>
const RevoluteMobilizer<T>& RevoluteMobilizer<T>::set_angle(
    systems::Context<T>* context, const T& angle) const {
  auto q = this->GetMutablePositions(context);
  DRAKE_ASSERT(q.size() == kNq);
  q[0] = angle;
  return *this;
}

template <typename T>
const T& RevoluteMobilizer<T>::get_angular_rate(
    const systems::Context<T>& context) const {
  const auto& v = this->get_velocities(context);
  DRAKE_ASSERT(v.size() == kNv);
  return v.coeffRef(0);
}

template <typename T>
const RevoluteMobilizer<T>& RevoluteMobilizer<T>::set_angular_rate(
    systems::Context<T>* context, const T& theta_dot) const {
  auto v = this->GetMutableVelocities(context);
  DRAKE_ASSERT(v.size() == kNv);
  v[0] = theta_dot;
  return *this;
}

template <typename T>
math::RigidTransform<T> RevoluteMobilizer<T>::CalcAcrossMobilizerTransform(
    const systems::Context<T>& context) const {
  const auto& q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == 1);
  const Eigen::AngleAxis<T> angle_axis(q[0], axis_F_);
  const math::RigidTransform<T> X_FM(angle_axis, Vector3<T>::Zero());
  return X_FM;
}

template <typename T>
SpatialVelocity<T> RevoluteMobilizer<T>::CalcAcrossMobilizerSpatialVelocity(
    const systems::Context<T>&,
    const Eigen::Ref<const VectorX<T>>& v) const {
  DRAKE_ASSERT(v.size() == kNv);
  return SpatialVelocity<T>(v[0] * axis_F_, Vector3<T>::Zero());
}

template <typename T>
SpatialAcceleration<T>
RevoluteMobilizer<T>::CalcAcrossMobilizerSpatialAcceleration(
    const systems::Context<T>&,
    const Eigen::Ref<const VectorX<T>>& vdot) const {
  DRAKE_ASSERT(vdot.size() == kNv);
  return SpatialAcceleration<T>(vdot[0] * axis_F_, Vector3<T>::Zero());
}

template <typename T>
void RevoluteMobilizer<T>::ProjectSpatialForce(
    const systems::Context<T>&,
    const SpatialForce<T>& F_Mo_F,
    Eigen::Ref<VectorX<T>> tau) const {
  DRAKE_ASSERT(tau.size() == kNv);
  // Computes tau = H_FMᵀ * F_Mo_F where H_FM ∈ ℝ⁶ is:
  // H_FM = [axis_Fᵀ; 0ᵀ]ᵀ (see CalcAcrossMobilizerSpatialVelocity().)
  // Therefore H_FMᵀ * F_Mo_F = axis_F.dot(F_Mo_F.translational()):
  tau[0] = axis_F_.dot(F_Mo_F.rotational());
}

template <typename T>
void RevoluteMobilizer<T>::DoCalcNMatrix(
    const systems::Context<T>&, EigenPtr<MatrixX<T>> N) const {
  (*N)(0, 0) = 1.0;
}

template <typename T>
void RevoluteMobilizer<T>::DoCalcNplusMatrix(
      const systems::Context<T>&, EigenPtr<MatrixX<T>> Nplus) const {
  (*Nplus)(0, 0) = 1.0;
}

template <typename T>
void RevoluteMobilizer<T>::MapVelocityToQDot(
    const systems::Context<T>&,
    const Eigen::Ref<const VectorX<T>>& v,
    EigenPtr<VectorX<T>> qdot) const {
  DRAKE_ASSERT(v.size() == kNv);
  DRAKE_ASSERT(qdot != nullptr);
  DRAKE_ASSERT(qdot->size() == kNq);
  *qdot = v;
}

template <typename T>
void RevoluteMobilizer<T>::MapQDotToVelocity(
    const systems::Context<T>&,
    const Eigen::Ref<const VectorX<T>>& qdot,
    EigenPtr<VectorX<T>> v) const {
  DRAKE_ASSERT(qdot.size() == kNq);
  DRAKE_ASSERT(v != nullptr);
  DRAKE_ASSERT(v->size() == kNv);
  *v = qdot;
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Mobilizer<ToScalar>>
RevoluteMobilizer<T>::TemplatedDoCloneToScalar(
    const MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& inboard_frame_clone =
      tree_clone.get_variant(this->inboard_frame());
  const Frame<ToScalar>& outboard_frame_clone =
      tree_clone.get_variant(this->outboard_frame());
  return std::make_unique<RevoluteMobilizer<ToScalar>>(
      inboard_frame_clone, outboard_frame_clone, this->revolute_axis());
}

template <typename T>
std::unique_ptr<Mobilizer<double>> RevoluteMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<AutoDiffXd>> RevoluteMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<symbolic::Expression>>
RevoluteMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::RevoluteMobilizer)
