#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "drake/multibody/parsing/frame_cache.h"
#include "drake/multibody/parsing/sdf_link.h"
#include "drake/multibody/parsing/sdf_model.h"
#include "drake/multibody/parsing/sdf_spec.h"

namespace drake {
namespace multibody {
namespace parsing {

/// Parses a single `<model>` from file a file named `sdf_path`.
/// A new SDFSpec object is created which will contain the single model from
/// the file.
std::unique_ptr<SDFSpec> ParseSDFModelFromFile(const std::string& sdf_path);

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
