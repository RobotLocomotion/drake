#include "drake/systems/plants/rigid_body_system/rigid_body_system.h"

// TODO(amcastro-tri): parsers are not "plants" and should therefore be moved
// somewhere else. Maybe inside "multibody_dynamics/parsers" when that exists.
#include "drake/systems/plants/parser_urdf.h"
#include "drake/common/eigen_autodiff_types.h"

using std::string;

using drake::parsers::ModelInstanceIdTable;

namespace drake {
namespace systems {

template <typename T>
RigidBodySystem<T>::RigidBodySystem() {
  penetration_stiffness_ = 150;
  penetration_damping_ = penetration_stiffness_ / 10.0;

  //this->DeclareInputPort(kVectorValued, length, kContinuousSampling);
  //this->DeclareOutputPort(kVectorValued, length, kContinuousSampling);

  // A default world with only the "world" body.
  multibody_world_ = std::make_unique<RigidBodyTree>();
}

template <typename T>
RigidBodySystem<T>::~RigidBodySystem() { }

template <typename T>
ModelInstanceIdTable RigidBodySystem<T>::AddModelInstanceFromUrdfFile(
    const string& filename,
    DrakeJoint::FloatingBaseType floating_base_type) {

  // Adds the URDF to the rigid body tree.
  ModelInstanceIdTable model_instance_id_table =
      drake::parsers::urdf::AddModelInstanceFromUrdfFile(
          filename, floating_base_type, nullptr, multibody_world_.get());

#if 0
  // Parses additional tags understood by rigid body system (e.g., actuators,
  // sensors, etc).
  XMLDocument xml_doc;
  xml_doc.LoadFile(filename.data());
  if (xml_doc.ErrorID() != XML_SUCCESS) {
    throw std::runtime_error(
        "RigidBodySystem::AddModelInstanceFromUrdfFile: ERROR: Failed to parse "
            "xml in file " + filename + "\n" + xml_doc.ErrorName());
  }

  ParseUrdf(*this, &xml_doc, model_instance_id_table);
#endif

  return model_instance_id_table;
}

// Explicitly instantiates on the most common scalar types.
template class DRAKE_RBS_EXPORT RigidBodySystem<double>;
//template class DRAKESYSTEMFRAMEWORK_EXPORT Gain<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
