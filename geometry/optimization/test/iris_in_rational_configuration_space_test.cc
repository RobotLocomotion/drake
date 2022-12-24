#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/iris.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/geometry/test_utilities/meshcat_environment.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace geometry {
namespace optimization {
namespace {

using Eigen::Vector2d;
using symbolic::Variable;
const double kInf = std::numeric_limits<double>::infinity();


// Helper method for testing IrisInConfigurationSpace from a urdf string.
HPolyhedron IrisFromUrdf(const std::string urdf,
                         const Eigen::Ref<const Eigen::VectorXd>& q_sample,
                         const Eigen::Ref<const Eigen::VectorXd>& q_star,
                         const IrisOptions& options) {
  systems::DiagramBuilder<double> builder;
  multibody::MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
  multibody::Parser parser(&plant);
  parser.package_map().AddPackageXml(FindResourceOrThrow(
      "drake/multibody/parsing/test/box_package/package.xml"));
  parser.AddModelsFromString(urdf, "urdf");
  plant.Finalize();
  auto diagram = builder.Build();

  auto context = diagram->CreateDefaultContext();
  plant.SetPositions(&plant.GetMyMutableContextFromRoot(context.get()),
                     q_sample);
  return IrisInRationalConfigurationSpace(
      plant, plant.GetMyContextFromRoot(*context), q_star, options);
}

GTEST_TEST(IrisInRationalConfigurationSpaceTest, DoublePendulum) {
  const double l1 = 2.0;
  const double l2 = 1.0;
  const double r = .5;
  const double w = 1.83;
  const std::string double_pendulum_urdf = fmt::format(
      R"(
<robot name="double_pendulum">
  <link name="fixed">
    <collision name="right">
      <origin rpy="0 0 0" xyz="{w_plus_one_half} 0 0"/>
      <geometry><box size="1 1 10"/></geometry>
    </collision>
    <collision name="left">
      <origin rpy="0 0 0" xyz="-{w_plus_one_half} 0 0"/>
      <geometry><box size="1 1 10"/></geometry>
    </collision>
  </link>
  <joint name="fixed_link_weld" type="fixed">
    <parent link="world"/>
    <child link="fixed"/>
  </joint>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57"/>
    <parent link="world"/>
    <child link="link1"/>
  </joint>
  <link name="link2">
    <collision name="ball">
      <origin rpy="0 0 0" xyz="0 0 -{l2}"/>
      <geometry><sphere radius="{r}"/></geometry>
    </collision>
  </link>
  <joint name="joint2" type="revolute">
    <origin rpy="0 0 0" xyz="0 0 -{l1}"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57"/>
    <parent link="link1"/>
    <child link="link2"/>
  </joint>
</robot>
)",
      fmt::arg("w_plus_one_half", w + .5), fmt::arg("l1", l1),
      fmt::arg("l2", l2), fmt::arg("r", r));

  const Vector2d q_sample = Vector2d::Zero();
  const Vector2d q_star = Vector2d::Ones();
  IrisOptions options;
  HPolyhedron region = IrisFromUrdf(double_pendulum_urdf, q_sample, q_star, options);

  // Note: You may use this to plot the solution in the desmos graphing
  // calculator link above.  Just copy each equation in the printed formula into
  // a desmos cell.  The intersection is the computed region.
  // const Vector2<symbolic::Expression> xy{symbolic::Variable("x"),
  //                                       symbolic::Variable("y")};
  // std::cout << (region.A()*xy <= region.b()) << std::endl;

  EXPECT_EQ(region.ambient_dimension(), 2);
  // Confirm that we've found a substantial region.
  EXPECT_GE(region.MaximumVolumeInscribedEllipsoid().Volume(), 1.5);

}
}  // namespace
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
