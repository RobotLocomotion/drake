#include "drake/examples/hsr/parameters/robot_parameters_loader.h"

#include "drake/common/find_resource.h"
#include "drake/common/yaml/yaml_read_archive.h"

namespace drake {
namespace examples {
namespace hsr {
namespace parameters {

using drake::yaml::YamlReadArchive;

bool ReadParametersFromFile(const std::string& filename,
                            const std::string& filepath_prefix,
                            RobotParameters<double>* parameters) {
  if (parameters != nullptr) {
    DRAKE_DEMAND(parameters->name == filename);
  }
  const std::string kExtension = ".yaml";
  const std::string filepath =
      FindResourceOrThrow(filepath_prefix + filename + kExtension);
  const YAML::Node& root = YAML::LoadFile(filepath);
  // Load PartParameters
  const YAML::Node& parts_parameters = root["parts_parameters"];
  for (const auto& part_iter : parts_parameters) {
    const std::string& part_name = part_iter.first.as<std::string>();
    PartParameters part_parameters(part_name);

    for (const auto joint_iter : part_iter.second) {
      JointParameters joint_parameters;
      YamlReadArchive(joint_iter).Accept(&joint_parameters);
      part_parameters.joints_parameters.push_back(joint_parameters);
    }
    parameters->parts_parameters.insert({part_name, part_parameters});
  }

  // Place holder to load CameraParameters.
  const YAML::Node& cameras_parameters = root["cameras_parameters"];
  for (const auto& camera_iter : cameras_parameters) {
    const std::string& camera_name = camera_iter.first.as<std::string>();
    CameraParameters<double> camera_parameters;
    camera_parameters.location.parent_frame_name =
        camera_iter.second["parent_frame_name"].as<std::string>();
    parameters->cameras_parameters.insert({camera_name, camera_parameters});
  }

  return true;
}

}  // namespace parameters
}  // namespace hsr
}  // namespace examples
}  // namespace drake
