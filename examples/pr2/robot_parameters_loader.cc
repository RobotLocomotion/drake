#include "drake/examples/pr2/robot_parameters_loader.h"

#include "drake/common/find_resource.h"
#include "drake/common/yaml/yaml_read_archive.h"

namespace drake {
namespace examples {
namespace pr2 {

using drake::yaml::YamlReadArchive;

bool ReadParametersFromFile(const std::string& filename,
                            const std::string& filepath_prefix,
                            RobotParameters* parameters) {
  DRAKE_DEMAND(parameters != nullptr);

  const std::string kExtension = ".yaml";
  const std::string filepath =
      FindResourceOrThrow(filepath_prefix + filename + kExtension);
  const YAML::Node& root = YAML::LoadFile(filepath);

  // Load PartParameters
  parameters->name = root["name"].as<std::string>();
  parameters->model_instance_info.model_name = parameters->name;
  parameters->model_instance_info.model_path =
      root["model_file_path"].as<std::string>();
  parameters->model_instance_info.parent_frame_name =
      root["parent_frame_name"].as<std::string>();
  parameters->model_instance_info.child_frame_name =
      root["child_frame_name"].as<std::string>();
  parameters->model_instance_info.is_floating_base =
      root["is_floating_base"].as<bool>();

  const YAML::Node& parts_parameters = root["parts_parameters"];
  for (const auto& part_iter : parts_parameters) {
    PartParameters part_parameters;
    YamlReadArchive(part_iter).Accept(&part_parameters);
    if (!part_parameters.IsValid()) {
      return false;
    } else {
      parameters->parts_parameters.insert(
          {part_parameters.name, part_parameters});
    }
  }

  // Final check the complete parameters are valid.
  if (parameters->IsValid()) {
    return true;
  } else {
    return false;
  }
}

}  // namespace pr2
}  // namespace examples
}  // namespace drake
