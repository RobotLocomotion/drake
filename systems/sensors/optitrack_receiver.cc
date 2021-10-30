#include "drake/systems/sensors/optitrack_receiver.h"

#include "optitrack/optitrack_frame_t.hpp"
#include <fmt/format.h>

namespace drake {
namespace systems {
namespace sensors {
namespace {

using math::RigidTransformd;

/* Returns the pose of rigid body frame `B` in Optitrack frame `O`. */
RigidTransformd Decode_X_OB(const optitrack::optitrack_rigid_body_t& body) {
  // The optitrack quaternion ordering is X-Y-Z-W and this needs fitting into
  // Eigen's W-X-Y-Z ordering.
  auto& q_xyzw = body.quat;
  const Eigen::Quaterniond q_wxyz(q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]);
  const Eigen::Vector3d position(body.xyz[0], body.xyz[1], body.xyz[2]);
  const math::RigidTransformd X_OB(q_wxyz, position);
  return X_OB;
}

}  // namespace

OptitrackReceiver::OptitrackReceiver(
    const std::map<int, std::string>& frame_map,
    const RigidTransformd& X_WO)
    : X_WO_(X_WO) {
  this->DeclareAbstractInputPort(
      "optitrack_frame_t",
      Value<optitrack::optitrack_frame_t>());
  for (const auto& [body_id, body_name] : frame_map) {
    this->DeclareAbstractOutputPort(
        body_name,
        []() { return AbstractValue::Make<RigidTransformd>(); },
        [this, body_id = body_id](const Context<double>& context,
                                  AbstractValue* output_abstract) {
          auto& output = output_abstract->get_mutable_value<RigidTransformd>();
          this->CalcOutput(context, body_id, &output);
        });
  }
}

void OptitrackReceiver::CalcOutput(
    const Context<double>& context, int body_id,
    RigidTransformd* output) const {
  const auto& input_port = this->get_input_port();
  const auto& message = input_port.Eval<optitrack::optitrack_frame_t>(context);
  for (const auto& body : message.rigid_bodies) {
    if (body.id == body_id) {
      *output = X_WO_ * Decode_X_OB(body);
      return;
    }
  }
  throw std::runtime_error(fmt::format(
      "OptitrackReceiver: input message does not contain body id={}", body_id));
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
