#include "drake/multibody/multibody_tree/test/free_rotating_body_plant.h"

#include <limits>

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace test {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;

template<typename T>
FreeRotatingBodyPlant<T>::FreeRotatingBodyPlant(double I, double J) :
    I_(I), J_(J) {
  BuildMultibodyTreeModel();
  DRAKE_DEMAND(model_.get_num_positions() == 3);
  DRAKE_DEMAND(model_.get_num_velocities() == 3);
  DRAKE_DEMAND(model_.get_num_states() == 6);
  this->DeclareContinuousState(
      model_.get_num_positions(),
      model_.get_num_velocities(), 0 /* num_z */);
}

template<typename T>
template<typename U>
FreeRotatingBodyPlant<T>::FreeRotatingBodyPlant(
    const FreeRotatingBodyPlant<U> &other) : FreeRotatingBodyPlant<T>(I_, J_) {}

template<typename T>
void FreeRotatingBodyPlant<T>::BuildMultibodyTreeModel() {
  UnitInertia<double> G_Bcm =
      UnitInertia<double>::AxiallySymmetric(J_, I_, Vector3<double>::UnitZ());
  const double kMass = 1.0;
  SpatialInertia<double> M_Bcm(kMass, Vector3<double>::Zero(), G_Bcm);

  body_ = &model_.template AddBody<RigidBody>(M_Bcm);

  mobilizer_ =
      &model_.template AddMobilizer<SpaceXYZMobilizer>(
          model_.get_world_frame(), body_->get_body_frame());

  model_.Finalize();
}

template<typename T>
std::unique_ptr<systems::LeafContext<T>>
FreeRotatingBodyPlant<T>::DoMakeContext() const {
  return model_.CreateDefaultContext();
}

template<typename T>
void FreeRotatingBodyPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T> &context,
    systems::ContinuousState<T> *derivatives) const {
  const auto x =
      dynamic_cast<const systems::BasicVector<T>&>(
          context.get_continuous_state_vector()).get_value();

  const int nq = model_.get_num_positions();
  const int nv = model_.get_num_velocities();

  MatrixX<T> M(nv, nv);
  model_.CalcMassMatrixViaInverseDynamics(context, &M);

  // Check if M is symmetric.
  const T err_sym = (M - M.transpose()).norm();
  DRAKE_DEMAND(err_sym < 10 * std::numeric_limits<double>::epsilon());

  VectorX<T> C(nv);
  model_.CalcBiasTerm(context, &C);

  auto v = x.bottomRows(nv);

  VectorX<T> qdot(nq);
  model_.MapVelocityToQDot(context, v, &qdot);

  VectorX<T> xdot(model_.get_num_states());

  xdot << qdot, M.llt().solve(-C);
  derivatives->SetFromVector(xdot);
}

template<typename T>
void FreeRotatingBodyPlant<T>::DoMapQDotToVelocity(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& qdot,
    systems::VectorBase<T>* generalized_velocity) const {
  const int nq = model_.get_num_positions();
  const int nv = model_.get_num_velocities();

  DRAKE_ASSERT(qdot.size() == nq);
  DRAKE_DEMAND(generalized_velocity != nullptr);
  DRAKE_DEMAND(generalized_velocity->size() == nv);

  VectorX<T> v(nv);
  model_.MapQDotToVelocity(context, qdot, &v);
  generalized_velocity->SetFromVector(v);
}

template<typename T>
void FreeRotatingBodyPlant<T>::DoMapVelocityToQDot(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& generalized_velocity,
    systems::VectorBase<T>* positions_derivative) const {
  const int nq = model_.get_num_positions();
  const int nv = model_.get_num_velocities();

  DRAKE_ASSERT(generalized_velocity.size() == nv);
  DRAKE_DEMAND(positions_derivative != nullptr);
  DRAKE_DEMAND(positions_derivative->size() == nq);

  VectorX<T> qdot(nq);
  model_.MapVelocityToQDot(context, generalized_velocity, &qdot);
  positions_derivative->SetFromVector(qdot);
}

template<typename T>
void FreeRotatingBodyPlant<T>::set_angular_velocity(
    systems::Context<T>* context, const Vector3<T>& w_WB) const {
  mobilizer_->set_angular_velocity(context, w_WB);
}

template<typename T>
Isometry3<T> FreeRotatingBodyPlant<T>::CalcPoseInWorldFrame(
    const systems::Context<T>& context) const {
  PositionKinematicsCache<T> pc(model_.get_topology());
  model_.CalcPositionKinematicsCache(context, &pc);
  return pc.get_X_WB(body_->get_node_index());
}

template<typename T>
SpatialVelocity<T> FreeRotatingBodyPlant<T>::CalcSpatialVelocityInWorldFrame(
    const systems::Context<T>& context) const {
  PositionKinematicsCache<T> pc(model_.get_topology());
  model_.CalcPositionKinematicsCache(context, &pc);
  VelocityKinematicsCache<T> vc(model_.get_topology());
  model_.CalcVelocityKinematicsCache(context, pc, &vc);
  return vc.get_V_WB(body_->get_node_index());
}

}  // namespace test
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::multibody_tree::test::FreeRotatingBodyPlant)
