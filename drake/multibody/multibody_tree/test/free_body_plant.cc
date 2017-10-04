#include "drake/multibody/multibody_tree/test/free_body_plant.h"

#include <cmath>
#include <vector>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/multibody/multibody_tree/rpy_mobilizer.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace test {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;

template<typename T>
FreeBodyPlant<T>::FreeBodyPlant(double I, double J) : I_(I), J_(J) {
  BuildMultibodyTreeModel();
  DRAKE_DEMAND(model_.get_num_positions() == 3);
  DRAKE_DEMAND(model_.get_num_velocities() == 3);
  DRAKE_DEMAND(model_.get_num_states() == 6);

  this->DeclareVectorOutputPort(
      systems::BasicVector<T>(model_.get_num_states()),
      &FreeBodyPlant::OutputState);
  this->DeclareContinuousState(
      model_.get_num_positions(),
      model_.get_num_velocities(), 0 /* num_z */);
}

template<typename T>
template<typename U>
FreeBodyPlant<T>::FreeBodyPlant(
    const FreeBodyPlant<U> &other) : FreeBodyPlant<T>(I_, J_) {}

template<typename T>
void FreeBodyPlant<T>::BuildMultibodyTreeModel() {
  UnitInertia<double> G_Bcm =
      UnitInertia<double>::AxiallySymmetric(J_, I_, Vector3<double>::UnitZ());
  SpatialInertia<double> M_Bcm(get_mass(), Vector3<double>::Zero(), G_Bcm);

  body_ = &model_.template AddBody<RigidBody>(M_Bcm);

  mobilizer_ =
      &model_.template AddMobilizer<RollPitchYawMobilizer>(
          model_.get_world_frame(), body_->get_body_frame());

  model_.Finalize();
}

template<typename T>
std::unique_ptr<systems::LeafContext<T>>
FreeBodyPlant<T>::DoMakeContext() const {
  return model_.CreateDefaultContext();
}

template<typename T>
void FreeBodyPlant<T>::OutputState(
    const systems::Context<T> &context,
    systems::BasicVector<T> *state_port_value) const {
  // Output port value is just the continuous state.
  const VectorX<T> state = context.get_continuous_state()->CopyToVector();
  state_port_value->SetFromVector(state);
}

template<typename T>
void FreeBodyPlant<T>::DoCalcTimeDerivatives(
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
void FreeBodyPlant<T>::set_angular_velocity(
    systems::Context<T>* context, const Vector3<T>& w_WB) const {
  mobilizer_->set_angular_velocity(context, w_WB);
}

template<typename T>
Isometry3<T> FreeBodyPlant<T>::CalcPoseInWorldFrame(
    const systems::Context<T>& context) const {
  PositionKinematicsCache<T> pc(model_.get_topology());
  model_.CalcPositionKinematicsCache(context, &pc);
  return pc.get_X_WB(body_->get_node_index());
}

template<typename T>
SpatialVelocity<T> FreeBodyPlant<T>::CalcSpatialVelocityInWorldFrame(
    const systems::Context<T>& context) const {
  PositionKinematicsCache<T> pc(model_.get_topology());
  model_.CalcPositionKinematicsCache(context, &pc);
  VelocityKinematicsCache<T> vc(model_.get_topology());
  model_.CalcVelocityKinematicsCache(context, pc, &vc);
  return vc.get_V_WB(body_->get_node_index());
}

template<typename T>
T FreeBodyPlant<T>::DoCalcKineticEnergy(
    const systems::Context<T>& context) const {
  const auto& mbt_context =
      dynamic_cast<const MultibodyTreeContext<T>&>(context);
  Eigen::VectorBlock<const VectorX<T>> v = mbt_context.get_velocities();

  const int nv = model_.get_num_velocities();

  MatrixX<T> M(nv, nv);
  model_.CalcMassMatrixViaInverseDynamics(context, &M);

  return 0.5 * v.transpose() * M * v;
}

template<typename T>
T FreeBodyPlant<T>::DoCalcPotentialEnergy(
    const systems::Context<T> &context) const {
  return 0.0;
}

template class FreeBodyPlant<double>;
template class FreeBodyPlant<AutoDiffXd>;

}  // namespace test
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
