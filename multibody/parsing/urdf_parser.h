#pragma once

#include <string>

#ifndef DRAKE_DOXYGEN_CXX
// TODO(jwnimmer-tri) Remove these forwarders on or about 2019-03-01.

#include "drake/common/drake_deprecated.h"
#include "drake/multibody/parsing/detail_urdf_parser.h"

namespace drake {
namespace multibody {
namespace parsing {

DRAKE_DEPRECATED(
    "AddModelFromUrdfFile is deprecated; please use the class "
    "drake::multibody::Parser instead.")
inline ModelInstanceIndex AddModelFromUrdfFile(
    const std::string& file_name,
    const std::string& model_name,
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr) {
  return detail::AddModelFromUrdfFile(
      file_name, model_name, plant, scene_graph);
}

DRAKE_DEPRECATED(
    "AddModelFromUrdfFile is deprecated; please use the class "
    "drake::multibody::Parser instead.")
inline ModelInstanceIndex AddModelFromUrdfFile(
    const std::string& file_name,
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph = nullptr) {
  return detail::AddModelFromUrdfFile(
      file_name, "", plant, scene_graph);
}

}  // namespace parsing
}  // namespace multibody
}  // namespace drake

#endif  // DRAKE_DOXYGEN_CXX
