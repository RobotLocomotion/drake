#pragma once

#include "drake/common/drake_path.h"
#include "drake/multibody/parsers/package_map.h"

namespace drake {
namespace examples {

/**
 * Adds example packages to @p package_map. This map can be used by the URDF
 * and SDF parsers to find ROS package resources referenced by URDF and SDF
 * files. Currently, the following packages are added:
 *
 *  - Atlas
 *  - IRB140
 *  - Valkyrie
 *  - hsrb_description
 *
 * @param[out] package_map A pointer to the PackageMap to which the exmamples
 * should be added. This must not be nullptr.
 */
inline void AddExamplePackages(parsers::PackageMap* package_map) {
  package_map->Add("Atlas", GetDrakePath() + "/examples/Atlas/");
  package_map->Add("IRB140", GetDrakePath() + "/examples/IRB140/");
  package_map->Add("Valkyrie", GetDrakePath() + "/examples/Valkyrie");
  package_map->Add("hsrb_description",
                   GetDrakePath() + "/../ros/hsrb_description");
}

}  // namespace examples
}  // namespace drake
