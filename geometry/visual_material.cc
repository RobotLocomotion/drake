#include "drake/geometry/visual_material.h"

namespace drake {
namespace geometry {

VisualMaterial::VisualMaterial() {}

VisualMaterial::VisualMaterial(const Eigen::Vector4d& diffuse)
    : diffuse_(diffuse) {}

}  // namespace geometry
}  // namespace drake
