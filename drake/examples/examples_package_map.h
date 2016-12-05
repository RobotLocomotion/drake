#pragma once

#include "drake/multibody/parser_common.h"

namespace drake {
namespace examples {

/**
 * Adds the example packages to @p package_map. This map can be used by the URDF
 * and SDF parsers to find package resources referenced by URDF and SDF files.
 * Currently, the following packages are added:
 *
 *  - Atlas
 *  - Valkyrie
 *  - hsrb_description
 *
 * @param[out] package_map A pointer to the PackageMap to which the exmamples
 * should be added. This pointer must not be nullptr.
 */
void AddExamplePackages(parsers::PackageMap* package_map);

}  // namespace examples
}  // namespace drake
