#include "drake/multibody/tree/screw_mobilizer.h"

#include <cmath>
#include <limits>

namespace drake {
namespace multibody {
namespace internal {

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
T ScrewMobilizer<T>::get_translation(
    const systems::Context<T>& context) const {
  auto q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);
  return get_screw_translation_from_rotation(q[0], screw_pitch_);
}

template <typename T>
const ScrewMobilizer<T>& ScrewMobilizer<T>::set_translation(
    systems::Context<T>* context,
    const T& translation) const {
  const double kEpsilon = std::sqrt(std::numeric_limits<double>::epsilon());
  using std::abs;
  DRAKE_THROW_UNLESS(abs(screw_pitch_) > kEpsilon ||
                     abs(translation) < kEpsilon);
  auto q = this->GetMutablePositions(context);
  DRAKE_ASSERT(q.size() == kNq);
  q[0] = get_screw_rotation_from_translation(translation, screw_pitch_);
  return *this;
}

template <typename T>
T ScrewMobilizer<T>::get_angle(
    const systems::Context<T>& context) const {
  auto q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);
  return q[0];
}

template <typename T>
const ScrewMobilizer<T>& ScrewMobilizer<T>::set_angle(
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
  return get_screw_translation_from_rotation(v[0], screw_pitch_);
}

template <typename T>
const ScrewMobilizer<T>& ScrewMobilizer<T>::set_translation_rate(
    systems::Context<T>* context,
    const T& vz) const {
  const double kEpsilon = std::sqrt(std::numeric_limits<double>::epsilon());
  using std::abs;
  DRAKE_THROW_UNLESS(abs(screw_pitch_) > kEpsilon || abs(vz) < kEpsilon);

  auto v = this->GetMutableVelocities(context);
  DRAKE_ASSERT(v.size() == kNv);
  v[0] = get_screw_rotation_from_translation(vz, screw_pitch_);
  return *this;
}

template <typename T>
T ScrewMobilizer<T>::get_angular_rate(
    const systems::Context<T>& context) const {
  auto v = this->get_velocities(context);
  DRAKE_ASSERT(v.size() == kNv);
  return v[0];
}

template <typename T>
const ScrewMobilizer<T>& ScrewMobilizer<T>::set_angular_rate(
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
  const Vector3<T> X_FM_translation(0.0, 0.0,
      get_screw_translation_from_rotation(q[0], screw_pitch_));
  return math::RigidTransform<T>(math::RotationMatrix<T>::MakeZRotation(q[0]),
                                 X_FM_translation);
}

template <typename T>
SpatialVelocity<T> ScrewMobilizer<T>::CalcAcrossMobilizerSpatialVelocity(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& v) const {
  DRAKE_ASSERT(v.size() == kNv);
  Vector6<T> V_FM_vector;
  V_FM_vector <<
    0.0, 0.0, get_screw_rotation_from_translation(v[0], screw_pitch_),
    0.0, 0.0, v[0];
  return SpatialVelocity<T>(V_FM_vector);
}

template <typename T>
SpatialAcceleration<T>
ScrewMobilizer<T>::CalcAcrossMobilizerSpatialAcceleration(
    const systems::Context<T>&,
    const Eigen::Ref<const VectorX<T>>& vdot) const {
  DRAKE_ASSERT(vdot.size() == kNv);
  Vector6<T> A_FM_vector;
  A_FM_vector << 0.0, 0.0, vdot[0], 0.0, 0.0,
                 get_screw_translation_from_rotation(vdot[0], screw_pitch_);
  return SpatialAcceleration<T>(A_FM_vector);
}

template <typename T>
void ScrewMobilizer<T>::ProjectSpatialForce(const systems::Context<T>&,
                                             const SpatialForce<T>& F_Mo_F,
                                             Eigen::Ref<VectorX<T>> tau) const {
  DRAKE_ASSERT(tau.size() == kNv);
  tau[0] = F_Mo_F.rotational()[2] + screw_pitch() * F_Mo_F.translational()[2];
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
                                                    outboard_frame_clone,
                                                    this->screw_pitch());
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
