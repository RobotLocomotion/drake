#include "drake/multibody/multibody_tree/prismatic_mobilizer.h"

#include <memory>
#include <stdexcept>

#include "drake/common/autodiff.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
const T& PrismaticMobilizer<T>::get_translation(
    const systems::Context<T>& context) const {
  const MultibodyTreeContext<T>& mbt_context =
      this->GetMultibodyTreeContextOrThrow(context);
  auto q = this->get_positions(mbt_context);
  DRAKE_ASSERT(q.size() == kNq);
  return q.coeffRef(0);
}

template <typename T>
const PrismaticMobilizer<T>& PrismaticMobilizer<T>::set_translation(
    systems::Context<T>* context, const T& translation) const {
  MultibodyTreeContext<T>& mbt_context =
      this->GetMutableMultibodyTreeContextOrThrow(context);
  auto q = this->get_mutable_positions(&mbt_context);
  DRAKE_ASSERT(q.size() == kNq);
  q[0] = translation;
  return *this;
}

template <typename T>
const T& PrismaticMobilizer<T>::get_translation_rate(
    const systems::Context<T> &context) const {
  const MultibodyTreeContext<T>& mbt_context =
      this->GetMultibodyTreeContextOrThrow(context);
  const auto& v = this->get_velocities(mbt_context);
  DRAKE_ASSERT(v.size() == kNv);
  return v.coeffRef(0);
}

template <typename T>
const PrismaticMobilizer<T>& PrismaticMobilizer<T>::set_translation_rate(
    systems::Context<T> *context, const T& translation_dot) const {
  MultibodyTreeContext<T>& mbt_context =
      this->GetMutableMultibodyTreeContextOrThrow(context);
  auto v = this->get_mutable_velocities(&mbt_context);
  DRAKE_ASSERT(v.size() == kNv);
  v[0] = translation_dot;
  return *this;
}

template <typename T>
void PrismaticMobilizer<T>::set_zero_state(const systems::Context<T>& context,
                                          systems::State<T>* state) const {
  // The default Mobilizer state of zero positions and velocities is used.
  this->set_default_zero_state(context, state);
}

template <typename T>
Isometry3<T> PrismaticMobilizer<T>::CalcAcrossMobilizerTransform(
    const MultibodyTreeContext<T>& context) const {
  return Isometry3<T>(
      Translation3<T>(get_translation(context) * translation_axis()));
}

template <typename T>
SpatialVelocity<T> PrismaticMobilizer<T>::CalcAcrossMobilizerSpatialVelocity(
    const MultibodyTreeContext<T>&,
    const Eigen::Ref<const VectorX<T>>& v) const {
  DRAKE_ASSERT(v.size() == kNv);
  return SpatialVelocity<T>(Vector3<T>::Zero(), v[0] * translation_axis());
}

template <typename T>
SpatialAcceleration<T>
PrismaticMobilizer<T>::CalcAcrossMobilizerSpatialAcceleration(
    const MultibodyTreeContext<T>&,
    const Eigen::Ref<const VectorX<T>>& vdot) const {
  DRAKE_ASSERT(vdot.size() == kNv);
  return SpatialAcceleration<T>(Vector3<T>::Zero(),
                                vdot[0] * translation_axis());
}

template <typename T>
void PrismaticMobilizer<T>::ProjectSpatialForce(
    const MultibodyTreeContext<T>&,
    const SpatialForce<T>& F_Mo_F,
    Eigen::Ref<VectorX<T>> tau) const {
  DRAKE_ASSERT(tau.size() == kNv);
  // Computes tau = H_FMᵀ * F_Mo_F where H_FM ∈ ℝ⁶ is:
  // H_FM = [0ᵀ; axis_Fᵀ]ᵀ (see CalcAcrossMobilizerSpatialVelocity().)
  // Therefore H_FMᵀ * F_Mo_F = axis_F.dot(F_Mo_F.translational()):
  tau[0] = axis_F_.dot(F_Mo_F.translational());
}

template <typename T>
void PrismaticMobilizer<T>::MapVelocityToQDot(
    const MultibodyTreeContext<T>&,
    const Eigen::Ref<const VectorX<T>>& v,
    EigenPtr<VectorX<T>> qdot) const {
  DRAKE_ASSERT(v.size() == kNv);
  DRAKE_ASSERT(qdot != nullptr);
  DRAKE_ASSERT(qdot->size() == kNq);
  *qdot = v;
}

template <typename T>
void PrismaticMobilizer<T>::MapQDotToVelocity(
    const MultibodyTreeContext<T>&,
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
PrismaticMobilizer<T>::TemplatedDoCloneToScalar(
    const MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& inboard_frame_clone =
      tree_clone.get_variant(this->inboard_frame());
  const Frame<ToScalar>& outboard_frame_clone =
      tree_clone.get_variant(this->outboard_frame());
  return std::make_unique<PrismaticMobilizer<ToScalar>>(
      inboard_frame_clone, outboard_frame_clone, this->translation_axis());
}

template <typename T>
std::unique_ptr<Mobilizer<double>> PrismaticMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<AutoDiffXd>> PrismaticMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

// Explicitly instantiates on the most common scalar types.
template class PrismaticMobilizer<double>;
template class PrismaticMobilizer<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
