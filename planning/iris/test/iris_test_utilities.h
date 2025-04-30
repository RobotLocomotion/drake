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

class IrisTestFixture : public ::testing::Test {
 protected:
  void SetUpEnvironment(const std::string& urdf);

  virtual void CheckRegion(
      const geometry::optimization::HPolyhedron& region) = 0;

  std::unique_ptr<CollisionChecker> checker_;
  multibody::MultibodyPlant<double>* plant_ptr_;
};

// One prismatic link with joint limits. Iris should return the joint limits.
class JointLimits1D : public IrisTestFixture {
 protected:
  JointLimits1D();

  void CheckRegion(const geometry::optimization::HPolyhedron& region);

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

// A simple double pendulum with link lengths `l1` and `l2` with a sphere at the
// tip of radius `r` between two (fixed) walls at `w` from the origin.  The
// true configuration space is - w + r ≤ l₁s₁ + l₂s₁₊₂ ≤ w - r.  These regions
// are visualized at https://www.desmos.com/calculator/ff0hbnkqhm.
class DoublePendulum : public IrisTestFixture {
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

// A block on a vertical track, free to rotate (in the plane) with width `w` of
// 2 and height `h` of 1, plus a ground plane at z=0.  The true configuration
// space is min(q₀ ± .5w sin(q₁) ± .5h cos(q₁)) ≥ 0, where the min is over the
// ±. This region is also visualized at
// https://www.desmos.com/calculator/ok5ckpa1kp.
class BlockOnGround : public IrisTestFixture {
 protected:
  BlockOnGround();

  void CheckRegion(const geometry::optimization::HPolyhedron& region);
  void PlotEnvironmentAndRegion(
      const geometry::optimization::HPolyhedron& region);

  std::shared_ptr<geometry::Meshcat> meshcat_;

  geometry::optimization::Hyperellipsoid starting_ellipsoid_;
  geometry::optimization::HPolyhedron domain_;

  inline static const std::string urdf_ = R"(
<robot name="block">
  <link name="fixed">
    <collision name="ground">
      <origin rpy="0 0 0" xyz="0 0 -1"/>
      <geometry><box size="10 10 2"/></geometry>
    </collision>
  </link>
  <joint name="fixed_link_weld" type="fixed">
    <parent link="world"/>
    <child link="fixed"/>
  </joint>
  <link name="link1"/>
  <joint name="joint1" type="prismatic">
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="3.0"/>
    <parent link="world"/>
    <child link="link1"/>
  </joint>
  <link name="link2">
    <collision name="block">
      <geometry><box size="2 1 1"/></geometry>
    </collision>
  </link>
  <joint name="joint2" type="revolute">
    <axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159"/>
    <parent link="link1"/>
    <child link="link2"/>
  </joint>
</robot>
)";
};

}  // namespace planning
}  // namespace drake
