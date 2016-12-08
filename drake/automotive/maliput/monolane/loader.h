#pragma once

#include <memory>
#include <string>

namespace drake {
namespace maliput {

namespace api {
class RoadGeometry;
}

namespace monolane {

/// @file
/// Loader for serialized monolane road networks.
///
/// The serialization is a fairly straightforward mapping of the Builder
/// interface onto YAML. See (TBD) for more detail of the format.
// TODO(maddog)  Describe format here.

/// Loads the input string as a maliput_monolane_builder document.
std::unique_ptr<const api::RoadGeometry> Load(const std::string& input);

/// Loads the named file as a maliput_monolane_builder document.
std::unique_ptr<const api::RoadGeometry> LoadFile(const std::string& filename);

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
