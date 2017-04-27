#include "drake/systems/sensors/gyroscope.h"

#include <memory>

#include "drake/systems/sensors/gyroscope_output.h"

using Eigen::Vector3d;
using Eigen::VectorXd;

using std::make_unique;

namespace drake {
namespace systems {
namespace sensors {

Gyroscope::Gyroscope(const std::string& name,
                     const RigidBodyFrame<double>& frame,
                     const RigidBodyTree<double>& tree)
    : name_(name),
      frame_(frame),
      tree_(tree) {
  input_port_index_ =
      DeclareInputPort(kVectorValued, tree_.get_num_positions() +
                                      tree_.get_num_velocities()).get_index();
  output_port_index_ = DeclareVectorOutputPort(
      GyroscopeOutput<double>()).get_index();
}

void Gyroscope::DoCalcOutput(const systems::Context<double>& context,
                             systems::SystemOutput<double>* output) const {
  // Obtains x, the RigidBodyPlant's state.
  const VectorXd x = this->EvalEigenVectorInput(context, input_port_index_);

  // Computes:
  //
  //  - q:    The RigidBodyPlant's position state vector.
  //  - v:    The RigidBodyPlant's velocity state vector.
  //
  // Note that x = [q, v].
  //
  const auto q = x.head(get_tree().get_num_positions());
  const auto v = x.tail(get_tree().get_num_velocities());

  // TODO(liang.fok): Obtain the KinematicsCache directly from the context
  // instead of recomputing it here.
  const KinematicsCache<double> cache = tree_.doKinematics(q, v);

  const drake::Vector6<double> V_W =
      tree_.CalcFrameSpatialVelocityInWorldFrame(cache, frame_);

  // Computes ω_WGo_W, the gyroscope's rotational velocity in the world frame.
  const auto w_WG_W = V_W.head<3>();

  const drake::Isometry3<double> X_WG =
      tree_.CalcFramePoseInWorldFrame(cache, frame_);

  // Extracts inverse rotation matrix from transform X_WG.
  const auto R_GW = X_WG.linear().transpose();

  // Computes ω_G, the gyroscope's rotational velocity in the gyroscope's
  // frame.
  const auto w_WG_G = R_GW * w_WG_W;

  // Saves the angular velocity values into the output port.
  BasicVector<double>* const output_vector =
      output->GetMutableVectorData(output_port_index_);
  output_vector->SetFromVector(w_WG_G);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
