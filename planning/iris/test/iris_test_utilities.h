#pragma once

#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

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

  std::vector<Vector1d> points_in_;
  std::vector<Vector1d> points_out_;

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

}  // namespace planning
}  // namespace drake
