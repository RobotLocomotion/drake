#pragma once

#include <string>
#include <vector>

#ifndef DRAKE_DOXYGEN_CXX
// TODO(jwnimmer-tri) Remove these forwarders on or about 2019-03-01.

#include "drake/common/drake_deprecated.h"
#include "drake/multibody/parsing/detail_path_utils.h"
#include "drake/multibody/parsing/detail_sdf_parser.h"

namespace drake {
namespace multibody {
namespace parsing {

DRAKE_DEPRECATED(
    "AddModelFromSdfFile is deprecated; please use the class "
    "drake::multibody::Parser instead.")
inline ModelInstanceIndex AddModelFromSdfFile(
    const std::string& file_name,
    const std::string& model_name,
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr) {
  multibody::PackageMap package_map;
  const std::string full_path = detail::GetFullPath(file_name);
  package_map.PopulateUpstreamToDrake(full_path);
  return detail::AddModelFromSdfFile(
      file_name, model_name, package_map, plant, scene_graph);
}

// Alternate version of AddModelFromSdfFile which always uses the "name"
// element from the model tag for the name of the newly created model instance.
DRAKE_DEPRECATED(
    "AddModelFromSdfFile is deprecated; please use the class "
    "drake::multibody::Parser instead.")
inline ModelInstanceIndex AddModelFromSdfFile(
    const std::string& file_name,
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr) {
  multibody::PackageMap package_map;
  const std::string full_path = detail::GetFullPath(file_name);
  package_map.PopulateUpstreamToDrake(full_path);
  return detail::AddModelFromSdfFile(
      file_name, "", package_map, plant, scene_graph);
}

DRAKE_DEPRECATED(
    "AddModelsFromSdfFile is deprecated; please use the class "
    "drake::multibody::Parser instead.")
inline std::vector<ModelInstanceIndex> AddModelsFromSdfFile(
    const std::string& file_name,
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr) {
  multibody::PackageMap package_map;
  const std::string full_path = detail::GetFullPath(file_name);
  package_map.PopulateUpstreamToDrake(full_path);
  return detail::AddModelsFromSdfFile(
      file_name, package_map, plant, scene_graph);
}

}  // namespace parsing
}  // namespace multibody
}  // namespace drake

#endif  // DRAKE_DOXYGEN_CXX
