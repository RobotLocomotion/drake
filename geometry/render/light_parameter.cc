#include "drake/geometry/render/light_parameter.h"

namespace drake {
namespace geometry {
namespace render {

std::ostream& operator<<(std::ostream& out, const LightType& t) {
  switch (t) {
    case LightType::kPoint:
      out << "point";
      break;
    case LightType::kSpot:
      out << "spot";
      break;
    case LightType::kDirectional:
      out << "directional";
      break;
  }
  return out;
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

std::ostream& operator<<(std::ostream& out, const LightFrame& t) {
  switch (t) {
    case LightFrame::kWorld:
      out << "world";
      break;
    case LightFrame::kCamera:
      out << "camera";
      break;
  }
  return out;
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
