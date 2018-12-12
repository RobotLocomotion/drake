#pragma once

#include <string>

#ifndef DRAKE_DOXYGEN_CXX
// TODO(jwnimmer-tri) Remove these forwarders on or about 2019-03-01.

#include "drake/common/drake_deprecated.h"
#include "drake/multibody/parsing/detail_urdf_parser.h"
#include "drake/multibody/parsing/package_map.h"

namespace drake {
namespace multibody {
namespace parsing {

DRAKE_DEPRECATED(
    "AddModelFromUrdfFile is deprecated; please use the class "
    "drake::multibody::Parser instead.")
inline ModelInstanceIndex AddModelFromUrdfFile(
    const std::string& file_name,
    const std::string& model_name,
    const PackageMap& package_map,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr) {
  return detail::AddModelFromUrdfFile(
      file_name, model_name, package_map, plant, scene_graph);
}

DRAKE_DEPRECATED(
    "AddModelFromUrdfFile is deprecated; please use the class "
    "drake::multibody::Parser instead.")
// Alternate version of AddModelFromUrdfFile which does not accept
// a PackageMap argument.
inline ModelInstanceIndex AddModelFromUrdfFile(
    const std::string& file_name,
    const std::string& model_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr) {
  return detail::AddModelFromUrdfFile(
      file_name, model_name, plant, scene_graph);
}

DRAKE_DEPRECATED(
    "AddModelFromUrdfFile is deprecated; please use the class "
    "drake::multibody::Parser instead.")
// Alternate version of AddModelFromUrdfFile which always uses the "name"
// element from the model tag for the name of the newly created model
// instance.
inline ModelInstanceIndex AddModelFromUrdfFile(
    const std::string& file_name,
    const PackageMap& package_map,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr) {
  return detail::AddModelFromUrdfFile(
      file_name, "", package_map, plant, scene_graph);
}

DRAKE_DEPRECATED(
    "AddModelFromUrdfFile is deprecated; please use the class "
    "drake::multibody::Parser instead.")
// Alternate version of AddModelFromUrdfFile which always uses the "name"
// element from the model tag for the name of the newly created model
// instance and does not accept a PackageMap argument.
inline ModelInstanceIndex AddModelFromUrdfFile(
    const std::string& file_name,
    multibody_plant::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr) {
  return detail::AddModelFromUrdfFile(
      file_name, "", plant, scene_graph);
}

}  // namespace parsing
}  // namespace multibody
}  // namespace drake

#endif  // DRAKE_DOXYGEN_CXX
