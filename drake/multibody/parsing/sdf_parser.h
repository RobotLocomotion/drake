#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "sdf/sdf.hh"

#include "drake/multibody/parsing/frame_cache.h"
#include "drake/multibody/parsing/sdf_link.h"
#include "drake/multibody/parsing/sdf_model.h"
#include "drake/multibody/parsing/sdf_spec.h"

namespace drake {
namespace multibody {
namespace parsing {

class SDFParser {
 public:
  SDFParser() {}

  /// Parses a single `<model>` from file a file named `sdf_path`.
  /// A new SDFSpec object is created which will contain the single model from
  /// the file.
  std::unique_ptr<SDFSpec> ParseSDFModelFromFile(const std::string& sdf_path);

 private:
  // Parses a <model> from the <sdf> element, referenced by
  // `sdf_model_element`.
  void ParseModel(::sdf::ElementPtr sdf_model_element, SDFSpec* spec);

  // Parses a new link from `sdf_link_element` (referencing a `<link>` element)
  // that gets added to `sdf_model`. It caches the the link's poses in
  // `sdf_model`.
  void ParseLink(const ::sdf::ElementPtr sdf_link_element, SDFModel* sdf_model);

  // Parses from `sdf_inertial_element` (referencing an `<inertial>` element)
  // the properties specified in an <inertial> element and adds them to `link`.
  void ParseInertial(::sdf::ElementPtr sdf_inertial_element, SDFLink* link);

  // Parses a new joint from the given SDF element and adds a SDFJoint to
  // `sdf_model`.
  // The model's frame cache is updated to "remember" the joint's frame pose.
  void ParseJoint(::sdf::ElementPtr sdf_joint_element, SDFModel* sdf_model);

  // Parses joint specific parameters according to their `<type>` tag.
  void ParseJointType(const ::sdf::ElementPtr sdf_joint_element,
                      const SDFModel& sdf_model,
                      SDFJoint* sdf_joint);
};

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
