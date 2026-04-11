#include "drake/geometry/render/light_parameter.h"

namespace drake {
namespace geometry {
namespace render {

std::string_view to_string(const LightType& t) {
  switch (t) {
    case LightType::kPoint:
      return "point";
    case LightType::kSpot:
      return "spot";
    case LightType::kDirectional:
      return "directional";
  }
  DRAKE_UNREACHABLE();
}

LightType light_type_from_string(const std::string& spec) {
  if (spec == "point") {
    return LightType::kPoint;
  } else if (spec == "spot") {
    return LightType::kSpot;
  } else if (spec == "directional") {
    return LightType::kDirectional;
  } else {
    throw std::runtime_error(
        fmt::format("Specified invalid light type: '{}'.", spec));
  }
}

std::string_view to_string(const LightFrame& f) {
  switch (f) {
    case LightFrame::kWorld:
      return "world";
    case LightFrame::kCamera:
      return "camera";
  }
  DRAKE_UNREACHABLE();
}

LightFrame light_frame_from_string(const std::string& spec) {
  if (spec == "world") {
    return LightFrame::kWorld;
  } else if (spec == "camera") {
    return LightFrame::kCamera;
  } else {
    throw std::runtime_error(
        fmt::format("Specified invalid light frame: '{}'.", spec));
  }
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
