#pragma once

#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/geometry/meshcat.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/multibody/rational/rational_forward_kinematics.h"
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

  const std::string urdf_ = R"(
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

  std::shared_ptr<geometry::Meshcat> meshcat_;

  geometry::optimization::Hyperellipsoid starting_ellipsoid_;
  geometry::optimization::HPolyhedron domain_;

  static constexpr double kPhysicalParamL1 = 2.0;
  static constexpr double kPhysicalParamL2 = 1.0;
  static constexpr double kPhysicalParamR = .5;
  static constexpr double kPhysicalParamW = 1.83;
  std::string urdf_;
};

class DoublePendulumRationalForwardKinematics : public DoublePendulum {
 protected:
  DoublePendulumRationalForwardKinematics();

  void CheckParameterization(
      const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>&
          parameterization);
  void CheckRegionRationalForwardKinematics(
      const geometry::optimization::HPolyhedron& region);
  void PlotEnvironmentAndRegionRationalForwardKinematics(
      const geometry::optimization::HPolyhedron& region,
      const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>&
          parameterization,
      const Eigen::Vector2d& region_query_point);

  geometry::optimization::Hyperellipsoid
      starting_ellipsoid_rational_forward_kinematics_;
  geometry::optimization::HPolyhedron domain_rational_forward_kinematics_;

  Eigen::Vector2d region_query_point_1_;
  Eigen::Vector2d region_query_point_2_;

  multibody::RationalForwardKinematics rational_kinematics_;
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

  const std::string urdf_ = R"(
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

// A (somewhat contrived) example of a concave configuration-space obstacle
// (resulting in a convex configuration-space, which we approximate with
// polytopes):  A simple pendulum of length `l` with a sphere at the tip of
// radius `r` on a vertical track, plus a ground plane at z=0.  The
// configuration space is given by the joint limits and z + l*cos(theta) >= r.
// The region is also visualized at
// https://www.desmos.com/calculator/flshvay78b. In addition to testing the
// convex space, this was originally a test for which Ibex found
// counter-examples that Snopt missed; now Snopt succeeds due to having
// options.num_collision_infeasible_samples > 1.
class ConvexConfigurationSpace : public IrisTestFixture {
 protected:
  ConvexConfigurationSpace();

  void CheckRegion(const geometry::optimization::HPolyhedron& region);
  void PlotEnvironment();
  void PlotRegion(const geometry::optimization::HPolyhedron& region);
  void PlotEnvironmentAndRegion(
      const geometry::optimization::HPolyhedron& region);

  std::shared_ptr<geometry::Meshcat> meshcat_;

  geometry::optimization::Hyperellipsoid starting_ellipsoid_;
  geometry::optimization::HPolyhedron domain_;

  // This point should be outside of the configuration space (in collision).
  // The particular value was found by visual inspection using meshcat.
  static constexpr double kZTest = 0.0;
  static constexpr double kThetaTest = -1.55;

  static constexpr double kPhysicalParamL = 1.5;
  static constexpr double kPhysicalParamR = 0.1;
  std::string urdf_;
};

// Another version of the ConvexConfigurationSpace test, adding the additional
// constraint that x <= -0.3.
class ConvexConfigurationSpaceWithThreadsafeConstraint
    : public ConvexConfigurationSpace {
 protected:
  ConvexConfigurationSpaceWithThreadsafeConstraint();

  void CheckRegion(const geometry::optimization::HPolyhedron& region);

  Eigen::Vector2d query_point_in_set_;
  Eigen::Vector2d query_point_not_in_set_;

  solvers::MathematicalProgram prog_;
};

// We also verify the code path when one of the additional constraints is not
// threadsafe. We construct the constraint (-2, -0.5) <= (x, y) <= (0, 1.5) in
// terms of the above struct IdentityConstraint, which is not tagged as
// threadsafe.
class ConvexConfigurationSpaceWithNotThreadsafeConstraint
    : public ConvexConfigurationSpaceWithThreadsafeConstraint {
 protected:
  ConvexConfigurationSpaceWithNotThreadsafeConstraint();
};

class ConvexConfigurationSubspace : public ConvexConfigurationSpace {
 protected:
  ConvexConfigurationSubspace();

  void CheckParameterization(
      const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>&
          parameterization);
  void CheckRegion(const geometry::optimization::HPolyhedron& region);
  void PlotEnvironmentAndRegionSubspace(
      const geometry::optimization::HPolyhedron& region,
      const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>&
          parameterization);

  Vector1d region_query_point_1_;
  Vector1d region_query_point_2_;
};

/* A movable sphere with fixed boxes in all corners.
┌───────────────┐
│┌────┐   ┌────┐│
││    │   │    ││
│└────┘   └────┘│
│       o       │
│┌────┐   ┌────┐│
││    │   │    ││
│└────┘   └────┘│
└───────────────┘ */
class FourCornersBoxes : public IrisTestFixture {
 protected:
  FourCornersBoxes();

  void CheckRegion(const geometry::optimization::HPolyhedron&);
  void CheckRegionContainsPoints(
      const geometry::optimization::HPolyhedron& region,
      const Eigen::Matrix2Xd& containment_points);
  void PlotEnvironmentAndRegion(
      const geometry::optimization::HPolyhedron& region);
  void PlotContainmentPoints(const Eigen::Matrix2Xd& containment_points);

  std::shared_ptr<geometry::Meshcat> meshcat_;

  geometry::optimization::Hyperellipsoid starting_ellipsoid_;
  geometry::optimization::HPolyhedron domain_;

  const std::string urdf_ = R"(
<robot name="boxes">
  <link name="fixed">
    <collision name="top_left">
      <origin rpy="0 0 0" xyz="-1 1 0"/>
      <geometry><box size="1.4 1.4 1.4"/></geometry>
    </collision>
    <collision name="top_right">
      <origin rpy="0 0 0" xyz="1 1 0"/>
      <geometry><box size="1.4 1.4 1.4"/></geometry>
    </collision>
    <collision name="bottom_left">
      <origin rpy="0 0 0" xyz="-1 -1 0"/>
      <geometry><box size="1.4 1.4 1.4"/></geometry>
    </collision>
    <collision name="bottom_right">
      <origin rpy="0 0 0" xyz="1 -1 0"/>
      <geometry><box size="1.4 1.4 1.4"/></geometry>
    </collision>
  </link>
  <joint name="fixed_link_weld" type="fixed">
    <parent link="world"/>
    <child link="fixed"/>
  </joint>
  <link name="movable">
    <collision name="sphere">
      <geometry><sphere radius="0.01"/></geometry>
    </collision>
  </link>
  <link name="for_joint"/>
  <joint name="x" type="prismatic">
    <axis xyz="1 0 0"/>
    <limit lower="-2" upper="2"/>
    <parent link="world"/>
    <child link="for_joint"/>
  </joint>
  <joint name="y" type="prismatic">
    <axis xyz="0 1 0"/>
    <limit lower="-2" upper="2"/>
    <parent link="for_joint"/>
    <child link="movable"/>
  </joint>
</robot>
)";
};

}  // namespace planning
}  // namespace drake
