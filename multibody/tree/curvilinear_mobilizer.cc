#include "drake/multibody/tree/curvilinear_mobilizer.h"

#include <memory>
#include <stdexcept>

#include "drake/multibody/tree/body_node_impl.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
CurvilinearMobilizer<T>::CurvilinearMobilizer(
    const SpanningForest::Mobod& mobod, const Frame<T>& inboard_frame_F,
    const Frame<T>& outboard_frame_M,
    const trajectories::PiecewiseConstantCurvatureTrajectory<double>&
        curvilinear_path)
    : MobilizerBase(mobod, inboard_frame_F, outboard_frame_M),
      curvilinear_path_(curvilinear_path) {}

template <typename T>
CurvilinearMobilizer<T>::~CurvilinearMobilizer() = default;

template <typename T>
std::unique_ptr<internal::BodyNode<T>> CurvilinearMobilizer<T>::CreateBodyNode(
    const internal::BodyNode<T>* parent_node, const RigidBody<T>* body,
    const Mobilizer<T>* mobilizer) const {
  return std::make_unique<internal::BodyNodeImpl<T, CurvilinearMobilizer>>(
      parent_node, body, mobilizer);
}

template <typename T>
std::string CurvilinearMobilizer<T>::position_suffix(
    int position_index_in_mobilizer) const {
  if (position_index_in_mobilizer == 0) {
    return "q";
  }
  throw std::runtime_error("CurvilinearMobilizer has only 1 position.");
}

template <typename T>
std::string CurvilinearMobilizer<T>::velocity_suffix(
    int velocity_index_in_mobilizer) const {
  if (velocity_index_in_mobilizer == 0) {
    return "v";
  }
  throw std::runtime_error("CurvilinearMobilizer has only 1 velocity.");
}

template <typename T>
const T& CurvilinearMobilizer<T>::get_distance(
    const drake::systems::Context<T>& context) const {
  auto q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);
  return q.coeffRef(0);
}

template <typename T>
const CurvilinearMobilizer<T>& CurvilinearMobilizer<T>::SetDistance(
    drake::systems::Context<T>* context, const T& distance) const {
  auto q = this->GetMutablePositions(context);
  DRAKE_ASSERT(q.size() == kNq);
  q[0] = distance;
  return *this;
}

template <typename T>
const T& CurvilinearMobilizer<T>::get_tangential_velocity(
    const drake::systems::Context<T>& context) const {
  const auto& v = this->get_velocities(context);
  DRAKE_ASSERT(v.size() == kNv);
  return v.coeffRef(0);
}

template <typename T>
const CurvilinearMobilizer<T>& CurvilinearMobilizer<T>::SetTangentialVelocity(
    drake::systems::Context<T>* context, const T& tangential_velocity) const {
  auto v = this->GetMutableVelocities(context);
  DRAKE_ASSERT(v.size() == kNv);
  v[0] = tangential_velocity;
  return *this;
}

template <typename T>
math::RigidTransform<T> CurvilinearMobilizer<T>::calc_X_FM(const T* q) const {
  return curvilinear_path_.CalcPose(*q);
}

template <typename T>
SpatialVelocity<T> CurvilinearMobilizer<T>::calc_V_FM(const T* q,
                                                      const T* v) const {
  return curvilinear_path_.CalcSpatialVelocity(*q, *v);
}

template <typename T>
SpatialVelocity<T> CurvilinearMobilizer<T>::calc_V_FM_M(
    const math::RigidTransform<T>&, const T* q, const T* v) const {
  return curvilinear_path_.CalcSpatialVelocityInM(*q, *v);
}

template <typename T>
SpatialAcceleration<T> CurvilinearMobilizer<T>::calc_A_FM(const T* q,
                                                          const T* v,
                                                          const T* vdot) const {
  return curvilinear_path_.CalcSpatialAcceleration(*q, *v, *vdot);
}

template <typename T>
SpatialAcceleration<T> CurvilinearMobilizer<T>::calc_A_FM_M(
    const math::RigidTransform<T>&, const T* q, const T* v,
    const T* vdot) const {
  return curvilinear_path_.CalcSpatialAccelerationInM(*q, *v, *vdot);
}

template <typename T>
void CurvilinearMobilizer<T>::calc_tau(const T* q,
                                       const SpatialForce<T>& F_BMo_F,
                                       T* tau) const {
  DRAKE_ASSERT(tau != nullptr);

  /* For this mobilizer, H_FM_F(q) = d/dv V_FM_F(q, v) is numerically equal to
   the spatial velocity V_FM_F(q, 1) evaluated at the unit velocity v = 1 [m/s]:
      tau = F_BMo_F.dot(V_FM_F(q, 1)) */
  const T v(1.);
  // Computes tau = H_FM_F(q)⋅F_BMo_F, equivalent to V_FM_F(q, 1)⋅F_BMo_F.
  tau[0] = calc_V_FM(q, &v).dot(F_BMo_F);
}

// Computes tau = H_FM_Mᵀ(q)⋅F_Mo_M
//              = [0 0 ρ(q) 1 0 0]⋅[t_Mᵀ f_Mᵀ]ᵀ
//              = ρ(q)⋅tz_M + fx_M
template <typename T>
void CurvilinearMobilizer<T>::calc_tau_from_M(const math::RigidTransform<T>&,
                                              const T* q,
                                              const SpatialForce<T>& F_BMo_M,
                                              T* tau) const {
  DRAKE_ASSERT(tau != nullptr);
  const T& rho = curvilinear_path_.curvature(*q);
  const Vector3<T>& t_M = F_BMo_M.rotational();
  const Vector3<T>& f_M = F_BMo_M.translational();
  tau[0] = rho * t_M[2] + f_M[0];
}

template <typename T>
math::RigidTransform<T> CurvilinearMobilizer<T>::CalcAcrossMobilizerTransform(
    const drake::systems::Context<T>& context) const {
  const auto& q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);
  return calc_X_FM(q.data());
}

template <typename T>
SpatialVelocity<T> CurvilinearMobilizer<T>::CalcAcrossMobilizerSpatialVelocity(
    const drake::systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& v) const {
  DRAKE_ASSERT(v.size() == kNv);
  const auto& q = this->get_positions(context);
  return calc_V_FM(q.data(), v.data());
}

template <typename T>
SpatialAcceleration<T>
CurvilinearMobilizer<T>::CalcAcrossMobilizerSpatialAcceleration(
    const drake::systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& vdot) const {
  DRAKE_ASSERT(vdot.size() == kNv);
  const auto& q = this->get_positions(context);
  const auto& v = this->get_velocities(context);
  return calc_A_FM(q.data(), v.data(), vdot.data());
}

template <typename T>
void CurvilinearMobilizer<T>::ProjectSpatialForce(
    const drake::systems::Context<T>& context, const SpatialForce<T>& F_BMo_F,
    Eigen::Ref<VectorX<T>> tau) const {
  DRAKE_ASSERT(tau.size() == kNv);
  const auto& q = this->get_positions(context);
  calc_tau(q.data(), F_BMo_F, tau.data());
}

template <typename T>
void CurvilinearMobilizer<T>::DoCalcNMatrix(const systems::Context<T>&,
                                            EigenPtr<MatrixX<T>> N) const {
  (*N)(0, 0) = 1.0;
}

template <typename T>
void CurvilinearMobilizer<T>::DoCalcNplusMatrix(
    const systems::Context<T>&, EigenPtr<MatrixX<T>> Nplus) const {
  (*Nplus)(0, 0) = 1.0;
}

template <typename T>
void CurvilinearMobilizer<T>::DoCalcNDotMatrix(
    const systems::Context<T>&, EigenPtr<MatrixX<T>> Ndot) const {
  (*Ndot)(0, 0) = 0.0;
}

template <typename T>
void CurvilinearMobilizer<T>::DoCalcNplusDotMatrix(
    const systems::Context<T>&, EigenPtr<MatrixX<T>> NplusDot) const {
  (*NplusDot)(0, 0) = 0.0;
}

template <typename T>
void CurvilinearMobilizer<T>::DoMapVelocityToQDot(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& v,
    EigenPtr<VectorX<T>> qdot) const {
  *qdot = v;
}

template <typename T>
void CurvilinearMobilizer<T>::DoMapQDotToVelocity(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& qdot,
    EigenPtr<VectorX<T>> v) const {
  *v = qdot;
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Mobilizer<ToScalar>>
CurvilinearMobilizer<T>::TemplatedDoCloneToScalar(
    const MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& inboard_frame_clone =
      tree_clone.get_variant(this->inboard_frame());
  const Frame<ToScalar>& outboard_frame_clone =
      tree_clone.get_variant(this->outboard_frame());
  return std::make_unique<CurvilinearMobilizer<ToScalar>>(
      tree_clone.get_mobod(this->mobod().index()), inboard_frame_clone,
      outboard_frame_clone,
      trajectories::PiecewiseConstantCurvatureTrajectory<double>(
          curvilinear_path_));
}

template <typename T>
std::unique_ptr<Mobilizer<double>> CurvilinearMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<AutoDiffXd>> CurvilinearMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<symbolic::Expression>>
CurvilinearMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::CurvilinearMobilizer);
