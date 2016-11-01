#pragma once

#include <memory>

#include <Eigen/Dense>

#include "drake/common/drake_export.h"
#include "drake/systems/plants/shapes/Geometry.h"

namespace DrakeShapes {
class DRAKE_EXPORT Element {
 public:
  Element(const Geometry& geometry_in,
          const Eigen::Isometry3d& T_element_to_local_in)
      : T_element_to_world(Eigen::Isometry3d::Identity()),
        T_element_to_local(T_element_to_local_in),
        geometry(geometry_in.clone()) {}

  explicit Element(const Geometry& geometry_in)
      : T_element_to_world(Eigen::Isometry3d::Identity()),
        T_element_to_local(Eigen::Isometry3d::Identity()),
        geometry(geometry_in.clone()) {}

  explicit Element(const Eigen::Isometry3d& T_element_to_local_in)
      : T_element_to_world(Eigen::Isometry3d::Identity()),
        T_element_to_local(T_element_to_local_in),
        geometry() {}

  virtual ~Element() {}

  virtual Element* clone() const;

  const Eigen::Isometry3d& getWorldTransform() const;

  const Eigen::Isometry3d& getLocalTransform() const;

  void SetLocalTransform(const Eigen::Isometry3d& T_element_to_local);

  virtual void updateWorldTransform(const Eigen::Isometry3d& T_local_to_world);

  Shape getShape() const;

  void setGeometry(const Geometry& geometry);

  bool hasGeometry() const;

  const Geometry& getGeometry() const;

  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void getTerrainContactPoints(Eigen::Matrix3Xd& points) const;

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

}  // namespace DrakeShapes
