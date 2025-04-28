#pragma once

#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/geometry/meshcat.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/planning/collision_checker.h"
#include "drake/planning/scene_graph_collision_checker.h"

namespace drake {
namespace planning {

class JointLimits1D : public ::testing::Test {
 protected:
  JointLimits1D();

  void CheckRegion(const geometry::optimization::HPolyhedron& region);

  std::unique_ptr<CollisionChecker> checker_;
  multibody::MultibodyPlant<double>* plant_ptr_;

  geometry::optimization::Hyperellipsoid starting_ellipsoid_;
  geometry::optimization::HPolyhedron domain_;

  inline static const std::string urdf_ = R"(
<robot name="limits">
  <link name="movable">
    <collision>
      <geometry><box size="1 1 1"/></geometry>
    </collision>
  </link>
  <joint name="movable" type="prismatic">
    <axis xyz="1 0 0"/>
    <limit lower="-2" upper="2"/>
    <parent link="world"/>
    <child link="movable"/>
  </joint>
</robot>
)";
};

class DoublePendulum : public ::testing::Test {
 protected:
  DoublePendulum();

  void CheckRegion(const geometry::optimization::HPolyhedron& region);
  void PlotEnvironmentAndRegion(
      const geometry::optimization::HPolyhedron& region);
  void PlotEnvironmentAndRegionRationalForwardKinematics(
      const geometry::optimization::HPolyhedron& region,
      const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>&
          parameterization,
      const Eigen::Vector2d& region_query_point);

  std::unique_ptr<CollisionChecker> checker_;
  multibody::MultibodyPlant<double>* plant_ptr_;
  std::shared_ptr<geometry::Meshcat> meshcat_;

  geometry::optimization::Hyperellipsoid starting_ellipsoid_;
  geometry::optimization::HPolyhedron domain_;

  geometry::optimization::Hyperellipsoid
      starting_ellipsoid_rational_forward_kinematics_;
  geometry::optimization::HPolyhedron domain_rational_forward_kinematics_;

  inline static const double physical_param_l1_ = 2.0;
  inline static const double physical_param_l2_ = 1.0;
  inline static const double physical_param_r_ = .5;
  inline static const double physical_param_w_ = 1.83;
  std::string urdf_;
};

}  // namespace planning
}  // namespace drake
