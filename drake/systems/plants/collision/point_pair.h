#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "Element.h"
#include "drake/drakeCollision_export.h"

namespace DrakeCollision {

class DRAKECOLLISION_EXPORT PointPair {
 public:
  PointPair() {}

  PointPair(const Element* elementA, const Element* elementB,
            const Eigen::Vector3d ptA, const Eigen::Vector3d ptB,
            const Eigen::Vector3d normal, double distance)
      : elementA_(elementA), elementB_(elementB),
        idA_(elementA->getId()), idB_(elementB->getId()),
        ptA_(ptA), ptB_(ptB),
        normal_(normal),
        distance_(distance) {}

  void getResults(Eigen::Vector3d* ptA, Eigen::Vector3d* ptB,
                  Eigen::Vector3d* normal) const;
  void getResults(Eigen::Vector3d* ptA, Eigen::Vector3d* ptB,
                  Eigen::Vector3d* normal, double* distance) const;
  ElementId getIdA() const;
  ElementId getIdB() const;
  const Element* const get_elementA() const;
  const Element* const get_elementB() const;
  Eigen::Vector3d getNormal() const;
  Eigen::Vector3d getPtA() const;
  Eigen::Vector3d getPtB() const;
  double getDistance() const;

  bool operator<(const PointPair& pt) const {
    return (distance_ < pt.distance_);
  }

  bool operator==(const PointPair& pt) const {
    return (distance_ == pt.distance_);
  }

  bool operator!=(const PointPair& pt) const {
    return (distance_ != pt.distance_);
  }

  bool operator<=(const PointPair& pt) const {
    return (distance_ <= pt.distance_);
  }

  bool operator>(const PointPair& pt) const {
    return (distance_ > pt.distance_);
  }

  bool operator>=(const PointPair& pt) const {
    return (distance_ >= pt.distance_);
  }

 protected:
  const Element* elementA_{nullptr};
  const Element* elementB_{nullptr};
  ElementId idA_{0};
  ElementId idB_{0};
  Eigen::Vector3d ptA_;
  Eigen::Vector3d ptB_;
  Eigen::Vector3d normal_;
  double distance_{};
};

}  // namespace DrakeCollision
