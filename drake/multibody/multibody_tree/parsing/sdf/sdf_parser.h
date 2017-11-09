#pragma once

#include <istream>
#include <memory>
#include <string>
#include <unordered_map>

#include "sdf/sdf.hh"

#include "drake/multibody/multibody_tree/parsing/sdf/frame_cache.h"
#include "drake/multibody/multibody_tree/parsing/sdf/sdf_model.h"
#include "drake/multibody/multibody_tree/parsing/sdf/sdf_spec.h"
#include "drake/multibody/multibody_tree/parsing/sdf/sdf_link.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace parsing {

class SDFParser {
 public:
  SDFParser() {}

  /// Parses a single `<model>` from file `sdf_file`.
  std::unique_ptr<SDFSpec> ParseSDFModelFromFile(
      const std::string& sdf_path);

 private:
  void ParseModel(sdf::ElementPtr sdf_model_element, SDFSpec* spec);

  // Parses from `sdf_link_element` a new link that gets added to `sdf_model`.
  // It caches the the link's poses.
  void ParseLink(const sdf::ElementPtr sdf_link_element, SDFModel* sdf_model);

  // Parses from `sdf_inertial_element` the properties specified in an
  // <inertial> element and adds them to `link`.
  void ParseInertial(sdf::ElementPtr sdf_inertial_element, SDFLink* link);

  // Parses a joint from the given SDF element and adds a SDFJoint to
  // `sdf_model`.
  // The model's frame cache is updated to "remember" the joint's frame pose.
  void ParseJoint(sdf::ElementPtr sdf_joint_element, SDFModel* sdf_model);

  // Parses joints by their particular joint type filling in addition fields
  // to fully specify a joint in `sdf_joint`.
  void ParseJointType(const sdf::ElementPtr sdf_joint_element,
                      const SDFModel& sdf_model,
                      SDFJoint* sdf_joint);
};

}  // namespace parsing
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
