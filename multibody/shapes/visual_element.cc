#include "drake/multibody/shapes/visual_element.h"

#include <string>

namespace DrakeShapes {

VisualElement::VisualElement(const Eigen::Isometry3d& T_element_to_local_in)
    : Element(T_element_to_local_in),
      material(Eigen::Vector4d(0.7, 0.7, 0.7, 1)) {}

VisualElement::VisualElement(const Geometry& geometry_in,
                             const Eigen::Isometry3d& T_element_to_local_in,
                             const Eigen::Vector4d& material_in,
                             const std::string& name_in)
    : Element(geometry_in, T_element_to_local_in), material(material_in),
      name(name_in) {}

void VisualElement::setMaterial(const Eigen::Vector4d& material_in) {
  material = material_in;
}

const Eigen::Vector4d& VisualElement::getMaterial() const {
  return material;
}

}  // namespace DrakeShapes
