#include "drake/multibody/tree/screw_mobilizer.h"

#include <cmath>
#include <limits>

#include "drake/multibody/tree/body_node_impl.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
ScrewMobilizer<T>::~ScrewMobilizer() = default;

template <typename T>
std::unique_ptr<BodyNode<T>> ScrewMobilizer<T>::CreateBodyNode(
    const BodyNode<T>* parent_node, const RigidBody<T>* body,
    const Mobilizer<T>* mobilizer) const {
  return std::make_unique<BodyNodeImpl<T, ScrewMobilizer>>(parent_node, body,
                                                           mobilizer);
}

template <typename T>
std::string ScrewMobilizer<T>::position_suffix(
    int position_index_in_mobilizer) const {
  if (position_index_in_mobilizer == 0) {
    return "q";
  }
  throw std::runtime_error("ScrewMobilizer has only 1 position.");
}

template <typename T>
std::string ScrewMobilizer<T>::velocity_suffix(
    int velocity_index_in_mobilizer) const {
  if (velocity_index_in_mobilizer == 0) {
    return "w";
  }
  throw std::runtime_error("ScrewMobilizer has only 1 velocity.");
}

template <typename T>
double ScrewMobilizer<T>::screw_pitch() const {
  return screw_pitch_;
}

template <typename T>
T ScrewMobilizer<T>::get_translation(const systems::Context<T>& context) const {
  auto q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);
  return GetScrewTranslationFromRotation(q[0], screw_pitch_);
}

template <typename T>
const ScrewMobilizer<T>& ScrewMobilizer<T>::SetTranslation(
    systems::Context<T>* context, const T& translation) const {
  const double kEpsilon = std::sqrt(std::numeric_limits<double>::epsilon());
  using std::abs;
  DRAKE_THROW_UNLESS(abs(screw_pitch_) > kEpsilon ||
                     abs(translation) < kEpsilon);
  auto q = this->GetMutablePositions(context);
  DRAKE_ASSERT(q.size() == kNq);
  q[0] = GetScrewRotationFromTranslation(translation, screw_pitch_);
  return *this;
}

template <typename T>
const T& ScrewMobilizer<T>::get_angle(
    const systems::Context<T>& context) const {
  auto q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);
  return q.coeffRef(0);
}

template <typename T>
const ScrewMobilizer<T>& ScrewMobilizer<T>::SetAngle(
    systems::Context<T>* context, const T& angle) const {
  auto q = this->GetMutablePositions(context);
  DRAKE_ASSERT(q.size() == kNq);
  q[0] = angle;
  return *this;
}

template <typename T>
T ScrewMobilizer<T>::get_translation_rate(
    const systems::Context<T>& context) const {
  auto v = this->get_velocities(context);
  DRAKE_ASSERT(v.size() == kNv);
  return GetScrewTranslationFromRotation(v[0], screw_pitch_);
}

template <typename T>
const ScrewMobilizer<T>& ScrewMobilizer<T>::SetTranslationRate(
    systems::Context<T>* context, const T& vz) const {
  const double kEpsilon = std::sqrt(std::numeric_limits<double>::epsilon());
  using std::abs;
  DRAKE_THROW_UNLESS(abs(screw_pitch_) > kEpsilon || abs(vz) < kEpsilon);

  auto v = this->GetMutableVelocities(context);
  DRAKE_ASSERT(v.size() == kNv);
  v[0] = GetScrewRotationFromTranslation(vz, screw_pitch_);
  return *this;
}

template <typename T>
const T& ScrewMobilizer<T>::get_angular_rate(
    const systems::Context<T>& context) const {
  auto v = this->get_velocities(context);
  DRAKE_ASSERT(v.size() == kNv);
  return v.coeffRef(0);
}

template <typename T>
const ScrewMobilizer<T>& ScrewMobilizer<T>::SetAngularRate(
    systems::Context<T>* context, const T& theta_dot) const {
  auto v = this->GetMutableVelocities(context);
  DRAKE_ASSERT(v.size() == kNv);
  v[0] = theta_dot;
  return *this;
}

template <typename T>
math::RigidTransform<T> ScrewMobilizer<T>::CalcAcrossMobilizerTransform(
    const systems::Context<T>& context) const {
  const auto& q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);
  return calc_X_FM(q.data());
}

template <typename T>
SpatialVelocity<T> ScrewMobilizer<T>::CalcAcrossMobilizerSpatialVelocity(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& v) const {
  DRAKE_ASSERT(v.size() == kNv);
  return calc_V_FM(nullptr, v.data());
}

template <typename T>
SpatialAcceleration<T>
ScrewMobilizer<T>::CalcAcrossMobilizerSpatialAcceleration(
    const systems::Context<T>&,
    const Eigen::Ref<const VectorX<T>>& vdot) const {
  DRAKE_ASSERT(vdot.size() == kNv);
  return calc_A_FM(nullptr, nullptr, vdot.data());
}

template <typename T>
void ScrewMobilizer<T>::ProjectSpatialForce(const systems::Context<T>&,
                                            const SpatialForce<T>& F_BMo_F,
                                            Eigen::Ref<VectorX<T>> tau) const {
  DRAKE_ASSERT(tau.size() == kNv);
  calc_tau(nullptr, F_BMo_F, tau.data());
}

template <typename T>
void ScrewMobilizer<T>::DoCalcNMatrix(const systems::Context<T>&,
                                      EigenPtr<MatrixX<T>> N) const {
  *N = Eigen::Matrix<T, 1, 1>::Identity();
}

template <typename T>
void ScrewMobilizer<T>::DoCalcNplusMatrix(const systems::Context<T>&,
                                          EigenPtr<MatrixX<T>> Nplus) const {
  *Nplus = Eigen::Matrix<T, 1, 1>::Identity();
}

template <typename T>
void ScrewMobilizer<T>::DoCalcNDotMatrix(const systems::Context<T>&,
                                         EigenPtr<MatrixX<T>> Ndot) const {
  (*Ndot)(0, 0) = 0.0;
}

template <typename T>
void ScrewMobilizer<T>::DoCalcNplusDotMatrix(
    const systems::Context<T>&, EigenPtr<MatrixX<T>> NplusDot) const {
  (*NplusDot)(0, 0) = 0.0;
}

template <typename T>
void ScrewMobilizer<T>::DoMapVelocityToQDot(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& v,
    EigenPtr<VectorX<T>> qdot) const {
  *qdot = v;
}

template <typename T>
void ScrewMobilizer<T>::DoMapQDotToVelocity(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& qdot,
    EigenPtr<VectorX<T>> v) const {
  *v = qdot;
}

template <typename T>
void ScrewMobilizer<T>::DoMapAccelerationToQDDot(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& vdot,
    EigenPtr<VectorX<T>> qddot) const {
  *qddot = vdot;
}

template <typename T>
void ScrewMobilizer<T>::DoMapQDDotToAcceleration(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& qddot,
    EigenPtr<VectorX<T>> vdot) const {
  *vdot = qddot;
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
  return std::make_unique<ScrewMobilizer<ToScalar>>(
      tree_clone.get_mobod(this->mobod().index()), inboard_frame_clone,
      outboard_frame_clone, this->screw_axis(), this->screw_pitch());
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
    class ::drake::multibody::internal::ScrewMobilizer);
