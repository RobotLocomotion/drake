#pragma once

#include <functional>
#include <istream>
#include <string>
#include <string_view>

#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

VolumeMesh<double> ReadVtkToVolumeMesh(
    const std::string& filename,
    std::function<void(std::string_view)> on_warning = {});

VolumeMesh<double> ReadVtkToVolumeMesh(
    std::istream* input_stream,
    std::function<void(std::string_view)> on_warning = {});

}  // namespace internal
}  // namespace geometry
}  // namespace drake
