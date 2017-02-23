#include "drake/systems/sensors/magnetometer.h"

#include <memory>

#include <Eigen/Dense>

#include "drake/math/quaternion.h"
#include "drake/systems/sensors/magnetometer_output.h"

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using std::make_unique;

namespace drake {
namespace systems {
namespace sensors {

Magnetometer::Magnetometer(const std::string& name,
                     const RigidBodyFrame<double>& frame,
                     const RigidBodyTree<double>& tree,
                     const Vector3d& north_vector)
    : name_(name),
      frame_(frame),
      tree_(tree),
      p_WoN_W_(north_vector) {
  input_port_index_ =
      DeclareInputPort(kVectorValued, tree_.get_num_positions() +
                                      tree_.get_num_velocities()).get_index();
  output_port_index_ =
      DeclareOutputPort(kVectorValued, 3).get_index();
}

std::unique_ptr<BasicVector<double>> Magnetometer::AllocateOutputVector(
    const OutputPortDescriptor<double>& descriptor) const {
  return make_unique<MagnetometerOutput<double>>();
}

void Magnetometer::DoCalcOutput(const systems::Context<double>& context,
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

  const Eigen::Isometry3d X_WM =
      tree_.CalcFramePoseInWorldFrame(cache, frame_);
  const auto R_MW = X_WM.linear().transpose();
  auto p_WoN_M = R_MW * p_WoN_W_;


  std::cout << "Magnetometer::DoCalcOutput():\n"
    << "  - p_WoN_W_ = " << p_WoN_W_.transpose() << "\n"
    << "  - X_WM =\n" << X_WM.matrix() << "\n"
    << "  - R_MW =\n" << R_MW.matrix() << "\n"
    << "  - p_WoN_M = " << p_WoN_M.transpose() << "\n"
    << std::endl;


  // This is the previous logic that assumes the world frame's +X axis is
  // pointing north.
  // Vector3d magnetic_north(1, 0, 0);
  // Vector4d quat_world_to_body = tree_.relativeQuaternion(
  //     cache, RigidBodyTreeConstants::kWorldBodyIndex,
  //     frame_.get_frame_index());
  // Vector3d mag_body = math::quatRotateVec(quat_world_to_body,
  //     magnetic_north);
  // std::cout << "mag_body = " << mag_body.transpose() << std::endl;

  // Saves the magnetometer reading into the output port.
  BasicVector<double>* const output_vector =
      output->GetMutableVectorData(output_port_index_);
  // output_vector->SetFromVector(mag_body);
  output_vector->SetFromVector(p_WoN_M);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
