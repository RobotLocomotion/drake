#pragma once

#include <memory>
#include <string>

#include "drake/multibody/parsing/sdf_spec.h"

namespace drake {
namespace multibody {
namespace parsing {

/// Parses a single `<model>` from file a file named `sdf_path`.
/// A new SdfSpec object is created which will contain the single model from
/// the file.
// TODO(amcastro-tri): throw an exception if more than one <model> is found.
std::unique_ptr<SdfSpec> ParseSdfModelFromFile(const std::string& sdf_path);

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
