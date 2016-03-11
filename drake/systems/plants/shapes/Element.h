#ifndef __DrakeShapesElement_H__
#define __DrakeShapesElement_H__

#include <memory>
#include <utility>
#include <stdint.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "Geometry.h"
#include "drake/drakeShapes_export.h"

namespace DrakeShapes {
class DRAKESHAPES_EXPORT Element {
 public:
  Element(const Geometry& geometry, const Eigen::Isometry3d& T_element_to_local)
      : T_element_to_world(Eigen::Isometry3d::Identity()),
        T_element_to_local(T_element_to_local),
        geometry(geometry.clone()){};

  Element(const Geometry& geometry)
      : T_element_to_world(Eigen::Isometry3d::Identity()),
        T_element_to_local(Eigen::Isometry3d::Identity()),
        geometry(geometry.clone()){};

  Element(const Eigen::Isometry3d& T_element_to_local)
      : T_element_to_world(Eigen::Isometry3d::Identity()),
        T_element_to_local(T_element_to_local),
        geometry(){};

  virtual ~Element(){};

  virtual Element* clone() const;

  const Eigen::Isometry3d& getWorldTransform() const;

  const Eigen::Isometry3d& getLocalTransform() const;

  void setLocalTransform(const Eigen::Isometry3d& T_element_to_local);

  virtual void updateWorldTransform(const Eigen::Isometry3d& T_local_to_world);

  Shape getShape() const;

  void setGeometry(const Geometry& geometry);

  bool hasGeometry() const;

  const Geometry& getGeometry() const;

  void getTerrainContactPoints(Eigen::Matrix3Xd& points);

 protected:
  virtual void setWorldTransform(const Eigen::Isometry3d& T_elem_to_world);
  Eigen::Isometry3d T_element_to_world;
  Eigen::Isometry3d T_element_to_local;
  std::unique_ptr<Geometry> geometry;

  Element(const Element&);
  Element& operator=(const Element&) { return *this; }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}
#endif
