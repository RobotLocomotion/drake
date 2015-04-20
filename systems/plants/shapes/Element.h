#ifndef __DrakeShapesElement_H__
#define __DrakeShapesElement_H__

#include <memory>
#include <utility>
#include <stdint.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "drakeShapesMacros.h"
#include "Geometry.h"

namespace DrakeShapes
{
  class DLLEXPORT_drakeShapes Element {
    public:
      Element(const Geometry& geometry, 
          const Eigen::Matrix4d& T_element_to_local = Eigen::Matrix4d::Identity())
        : geometry(geometry.clone()), T_element_to_local(T_element_to_local)
      {};

      Element(const Eigen::Matrix4d& T_element_to_local = Eigen::Matrix4d::Identity())
        : geometry(), T_element_to_local(T_element_to_local)
      {};

      virtual ~Element(){};

      virtual Element* clone() const;

      const Eigen::Matrix4d& getWorldTransform() const; 

      const Eigen::Matrix4d& getLocalTransform() const; 

      virtual void updateWorldTransform(const Eigen::Matrix4d& T_local_to_world);

      const Shape getShape() const;

      void setGeometry(const Geometry& geometry);

      bool hasGeometry() const;

      const Geometry& getGeometry() const;
      
      void getTerrainContactPoints(Eigen::Matrix3Xd &points);

    protected:

      virtual void setWorldTransform(const Eigen::Matrix4d& T_elem_to_world);
      Eigen::Matrix4d T_element_to_world;
      const Eigen::Matrix4d T_element_to_local;
      std::unique_ptr<Geometry> geometry;

      Element(const Element&);
      Element& operator=(const Element&) { return *this; }
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
#endif
