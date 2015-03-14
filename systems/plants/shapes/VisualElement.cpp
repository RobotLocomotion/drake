#include "VisualElement.h"

namespace DrakeShapes
{
  //VisualElement::VisualElement(unique_ptr<Geometry> geometry,
                               //const Matrix4d& T_element_to_local,
                               //const Vector4d& material)
    //: Element(move(geometry), T_element_to_local), material(material) 
  //{}

  void VisualElement::setMaterial(const Eigen::Vector4d& material)
  {
    this->material = material;
  }

  const Eigen::Vector4d& VisualElement::getMaterial() const
  {
    return this->material;
  }
}
