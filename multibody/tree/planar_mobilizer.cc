#include "drake/multibody/tree/planar_mobilizer.h"

#include <memory>
#include <string>

#include "drake/common/eigen_types.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/tree/body_node_impl.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
PlanarMobilizer<T>::~PlanarMobilizer() = default;

template <typename T>
std::unique_ptr<BodyNode<T>> PlanarMobilizer<T>::CreateBodyNode(
    const BodyNode<T>* parent_node, const RigidBody<T>* body,
    const Mobilizer<T>* mobilizer) const {
  return std::make_unique<BodyNodeImpl<T, PlanarMobilizer>>(parent_node, body,
                                                            mobilizer);
}

template <typename T>
std::string PlanarMobilizer<T>::position_suffix(
    int position_index_in_mobilizer) const {
  switch (position_index_in_mobilizer) {
    case 0:
      return "x";
    case 1:
      return "y";
    case 2:
      return "qz";
  }
  throw std::runtime_error("PlanarMobilizer has only 3 positions.");
}

template <typename T>
std::string PlanarMobilizer<T>::velocity_suffix(
    int velocity_index_in_mobilizer) const {
  switch (velocity_index_in_mobilizer) {
    case 0:
      return "vx";
    case 1:
      return "vy";
    case 2:
      return "wz";
  }
  throw std::runtime_error("PlanarMobilizer has only 3 velocities.");
}

template <typename T>
Vector2<T> PlanarMobilizer<T>::get_translations(
    const systems::Context<T>& context) const {
  auto q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);
  return q.head(2);
}

template <typename T>
const PlanarMobilizer<T>& PlanarMobilizer<T>::set_translations(
    systems::Context<T>* context,
    const Eigen::Ref<const Vector2<T>>& translations) const {
  auto q = this->GetMutablePositions(context);
  DRAKE_ASSERT(q.size() == kNq);
  q.head(2) = translations;
  return *this;
}

template <typename T>
const T& PlanarMobilizer<T>::get_angle(
    const systems::Context<T>& context) const {
  auto q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);
  return q.coeffRef(2);
}

template <typename T>
const PlanarMobilizer<T>& PlanarMobilizer<T>::SetAngle(
    systems::Context<T>* context, const T& angle) const {
  auto q = this->GetMutablePositions(context);
  DRAKE_ASSERT(q.size() == kNq);
  q[2] = angle;
  return *this;
}

template <typename T>
Vector2<T> PlanarMobilizer<T>::get_translation_rates(
    const systems::Context<T>& context) const {
  const auto& v = this->get_velocities(context);
  DRAKE_ASSERT(v.size() == kNv);
  return v.head(2);
}

template <typename T>
const PlanarMobilizer<T>& PlanarMobilizer<T>::SetTranslationRates(
    systems::Context<T>* context,
    const Eigen::Ref<const Vector2<T>>& v_FM_F) const {
  auto v = this->GetMutableVelocities(context);
  DRAKE_ASSERT(v.size() == kNv);
  v.head(2) = v_FM_F;
  return *this;
}

template <typename T>
const T& PlanarMobilizer<T>::get_angular_rate(
    const systems::Context<T>& context) const {
  const auto& v = this->get_velocities(context);
  DRAKE_ASSERT(v.size() == kNv);
  return v.coeffRef(2);
}

template <typename T>
const PlanarMobilizer<T>& PlanarMobilizer<T>::SetAngularRate(
    systems::Context<T>* context, const T& theta_dot) const {
  auto v = this->GetMutableVelocities(context);
  DRAKE_ASSERT(v.size() == kNv);
  v[2] = theta_dot;
  return *this;
}

template <typename T>
math::RigidTransform<T> PlanarMobilizer<T>::CalcAcrossMobilizerTransform(
    const systems::Context<T>& context) const {
  const auto& q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);
  return calc_X_FM(q.data());
}

template <typename T>
SpatialVelocity<T> PlanarMobilizer<T>::CalcAcrossMobilizerSpatialVelocity(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& v) const {
  DRAKE_ASSERT(v.size() == kNv);
  return calc_V_FM(nullptr, v.data());
}

template <typename T>
SpatialAcceleration<T>
PlanarMobilizer<T>::CalcAcrossMobilizerSpatialAcceleration(
    const systems::Context<T>&,
    const Eigen::Ref<const VectorX<T>>& vdot) const {
  DRAKE_ASSERT(vdot.size() == kNv);
  return calc_A_FM(nullptr, nullptr, vdot.data());
}

template <typename T>
void PlanarMobilizer<T>::ProjectSpatialForce(const systems::Context<T>&,
                                             const SpatialForce<T>& F_BMo_F,
                                             Eigen::Ref<VectorX<T>> tau) const {
  DRAKE_ASSERT(tau.size() == kNv);
  calc_tau(nullptr, F_BMo_F, tau.data());
}

template <typename T>
void PlanarMobilizer<T>::DoCalcNMatrix(const systems::Context<T>&,
                                       EigenPtr<MatrixX<T>> N) const {
  *N = Matrix3<T>::Identity();
}

template <typename T>
void PlanarMobilizer<T>::DoCalcNplusMatrix(const systems::Context<T>&,
                                           EigenPtr<MatrixX<T>> Nplus) const {
  *Nplus = Matrix3<T>::Identity();
}

template <typename T>
void PlanarMobilizer<T>::DoCalcNDotMatrix(const systems::Context<T>&,
                                          EigenPtr<MatrixX<T>> Ndot) const {
  *Ndot = Matrix3<T>::Zero();
}

template <typename T>
void PlanarMobilizer<T>::DoCalcNplusDotMatrix(
    const systems::Context<T>&, EigenPtr<MatrixX<T>> NplusDot) const {
  *NplusDot = Matrix3<T>::Zero();
}

template <typename T>
void PlanarMobilizer<T>::DoMapVelocityToQDot(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& v,
    EigenPtr<VectorX<T>> qdot) const {
  *qdot = v;
}

template <typename T>
void PlanarMobilizer<T>::DoMapQDotToVelocity(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& qdot,
    EigenPtr<VectorX<T>> v) const {
  *v = qdot;
}

template <typename T>
void PlanarMobilizer<T>::DoMapAccelerationToQDDot(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& vdot,
    EigenPtr<VectorX<T>> qddot) const {
  *qddot = vdot;
}

template <typename T>
void PlanarMobilizer<T>::DoMapQDDotToAcceleration(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& qddot,
    EigenPtr<VectorX<T>> vdot) const {
  *vdot = qddot;
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Mobilizer<ToScalar>>
PlanarMobilizer<T>::TemplatedDoCloneToScalar(
    const MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& inboard_frame_clone =
      tree_clone.get_variant(this->inboard_frame());
  const Frame<ToScalar>& outboard_frame_clone =
      tree_clone.get_variant(this->outboard_frame());
  return std::make_unique<PlanarMobilizer<ToScalar>>(
      tree_clone.get_mobod(this->mobod().index()), inboard_frame_clone,
      outboard_frame_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<double>> PlanarMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<AutoDiffXd>> PlanarMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<symbolic::Expression>>
PlanarMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::PlanarMobilizer);
