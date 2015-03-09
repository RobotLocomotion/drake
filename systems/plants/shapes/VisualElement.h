#ifndef __DrakeShapesVisualElement_H__
#define __DrakeShapesVisualElement_H__
#include <memory>

#include <Eigen/Dense>

#include "Element.h"

#include "drakeShapesMacros.h"

namespace DrakeShapes
{
  class DLLEXPORT_drakeShapes VisualElement : public Element
  {
    public:
      VisualElement(std::unique_ptr<DrakeShapes::Geometry> geometry,
                    const Eigen::Matrix4d& T_element_to_local, 
                    const Eigen::Vector4d& material)
    : Element(move(geometry), T_element_to_local), material(material) {};

      const Eigen::Vector4d& getMaterial() const;
    protected:
      Eigen::Vector4d material;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
#endif
