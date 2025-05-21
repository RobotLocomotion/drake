#include "drake/multibody/tree/element_collection.h"

#include "drake/multibody/tree/element_collection-inl.h"
#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/joint_actuator.h"
#include "drake/multibody/tree/model_instance.h"
#include "drake/multibody/tree/rigid_body.h"

namespace drake {
namespace multibody {
namespace internal {

using symbolic::Expression;

template class ElementCollection<double, Frame, FrameIndex>;
template class ElementCollection<AutoDiffXd, Frame, FrameIndex>;
template class ElementCollection<Expression, Frame, FrameIndex>;

template class ElementCollection<double, Joint, JointIndex>;
template class ElementCollection<AutoDiffXd, Joint, JointIndex>;
template class ElementCollection<Expression, Joint, JointIndex>;

template class ElementCollection<double, JointActuator, JointActuatorIndex>;
template class ElementCollection<AutoDiffXd, JointActuator, JointActuatorIndex>;
template class ElementCollection<Expression, JointActuator, JointActuatorIndex>;

template class ElementCollection<double, ModelInstance, ModelInstanceIndex>;
template class ElementCollection<AutoDiffXd, ModelInstance, ModelInstanceIndex>;
template class ElementCollection<Expression, ModelInstance, ModelInstanceIndex>;

template class ElementCollection<double, RigidBody, BodyIndex>;
template class ElementCollection<AutoDiffXd, RigidBody, BodyIndex>;
template class ElementCollection<Expression, RigidBody, BodyIndex>;

}  // namespace internal
}  // namespace multibody
}  // namespace drake
