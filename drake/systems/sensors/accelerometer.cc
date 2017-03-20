#include "drake/systems/sensors/accelerometer.h"

#include <cmath>
#include <vector>

#include "drake/math/quaternion.h"
#include "drake/systems/sensors/accelerometer_output.h"

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using std::make_unique;
using std::move;
using std::string;

namespace drake {
namespace systems {
namespace sensors {

Accelerometer::Accelerometer(const string& name,
                             const RigidBodyFrame<double>& frame,
                             const RigidBodyTree<double>& tree,
                             bool include_gravity)
    : name_(name),
      frame_(frame),
      tree_(tree),
      include_gravity_(include_gravity) {
  plant_state_input_port_index_ =
      DeclareInputPort(kVectorValued, tree_.get_num_positions() +
                                      tree_.get_num_velocities()).get_index();
  plant_state_derivative_input_port_index_ =
      DeclareInputPort(kVectorValued, tree_.get_num_positions() +
                                      tree_.get_num_velocities()).get_index();
  output_port_index_ = DeclareVectorOutputPort(
      AccelerometerOutput<double>()).get_index();
}

Accelerometer* Accelerometer::AttachAccelerometer(
    const string& name,
    const RigidBodyFrame<double>& frame,
    const RigidBodyPlant<double>& plant,
    bool include_gravity,
    DiagramBuilder<double>* builder) {
  const RigidBodyTree<double>& tree = plant.get_rigid_body_tree();

  // Ensures the input parameters are valid.
  {
    DRAKE_DEMAND(builder != nullptr);
    bool plant_in_builder = false;
    std::vector<systems::System<double>*> systems =
        builder->GetMutableSystems();
    for (auto system : systems) {
      if (system == &plant) {
        plant_in_builder = true;
      }
    }
    if (!plant_in_builder) {
      throw std::runtime_error("Accelerometer::AttachAccelerometer: ERROR: The "
          "provide DiagramBuilder does not contain the provided "
          "RigidBodyPlant.");
    }
  }

  auto accelerometer = builder->template AddSystem<Accelerometer>(name, frame,
      tree, include_gravity);
  builder->Connect(plant.state_output_port(),
      accelerometer->get_plant_state_input_port());
  // TODO(liang.fok) Connect the accelerometer's plant state derivative input
  // port once RigidBodyPlant has an output port containing xdot. This can only
  // be done once #2890 is resolved.
  return accelerometer;
}

void Accelerometer::DoCalcOutput(const systems::Context<double>& context,
                                 systems::SystemOutput<double>* output) const {
  // Obtains the two input vectors, x and xdot. x is the state vector. xdot is
  // the time derivative of x.
  const VectorXd x = this->EvalEigenVectorInput(
    context, plant_state_input_port_index_);
  const VectorXd xdot = this->EvalEigenVectorInput(
      context, plant_state_derivative_input_port_index_);

  // Computes:
  //
  //  - q:    The RigidBodyPlant's position state vector.
  //  - v:    The RigidBodyPlant's velocity state vector.
  //  - vdot: The derivative of v, which is the RigidBodyPlant's acceleration
  //          vector.
  //
  // Note that x = [q, v].
  //
  const auto q = x.head(get_tree().get_num_positions());
  const auto v = x.tail(get_tree().get_num_velocities());
  const auto vdot = xdot.tail(get_tree().get_num_velocities());

  // TODO(liang.fok): Obtain the KinematicsCache directly from the context
  // instead of recomputing it here.
  const KinematicsCache<double> kinematics_cache =
      tree_.doKinematics(q, v);

  const auto J_WF = tree_.CalcFrameSpatialVelocityJacobianInWorldFrame(
      kinematics_cache, frame_, false /* in_terms_of_qdot */);

  const drake::Vector6<double> Jdot_WF_times_v =
      tree_.CalcFrameSpatialVelocityJacobianDotTimesVInWorldFrame(
          kinematics_cache, frame_);

  // Note that an "A" in the variable name denotes "spatial acceleration" while
  // an "a" denotes "linear acceleration". For more details about this
  // nomenclature, see the website linked to below.
  //
  // http://drake.mit.edu/doxygen_cxx/group__multibody__spatial__vectors.html
  //
  const auto A_WF = Jdot_WF_times_v + J_WF * vdot;

  drake::Isometry3<double> X_WF = tree_.CalcFramePoseInWorldFrame(
      kinematics_cache, frame_);

  // Extracts inverse rotation matrix from transform, and translational
  // part of spatial acceleration.
  auto R_FW = X_WF.linear().transpose();
  auto a_WF_W = A_WF.tail<3>();     // Emphasizing expressed in W.
  Vector3d a_WF_F = R_FW * a_WF_W;  // Re-expresses acceleration in F.

  if (include_gravity_) {
    const Vector3d gravity_W = tree_.a_grav.tail<3>();
    a_WF_F += R_FW * gravity_W;
  }

  // Saves the acceleration readings into the output port.
  BasicVector<double>* output_vector =
      output->GetMutableVectorData(output_port_index_);

  output_vector->SetFromVector(a_WF_F);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
