#include "drake/planning/iris/test/iris_test_utilities.h"

#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/planning/collision_checker.h"
#include "drake/planning/robot_diagram_builder.h"

namespace drake {
namespace planning {

using geometry::optimization::HPolyhedron;
using geometry::optimization::Hyperellipsoid;

JointLimits1D::JointLimits1D() {
  CollisionCheckerParams params;
  RobotDiagramBuilder<double> builder(0.0);

  params.robot_model_instances =
      builder.parser().AddModelsFromString(urdf_, "urdf");

  plant_ptr_ = &(builder.plant());
  plant_ptr_->Finalize();

  params.model = builder.Build();
  params.edge_step_size = 0.01;
  checker_ = std::make_unique<SceneGraphCollisionChecker>(std::move(params));

  Vector1d sample = Vector1d::Zero(1);
  starting_ellipsoid_ = Hyperellipsoid::MakeHypersphere(1e-2, sample);
  domain_ = HPolyhedron::MakeBox(plant_ptr_->GetPositionLowerLimits(),
                                 plant_ptr_->GetPositionUpperLimits());
}

void JointLimits1D::CheckRegion(const HPolyhedron& region) {
  EXPECT_EQ(region.ambient_dimension(), 1);

  const double eps = 1e-5;
  const double qmin = -2.0, qmax = 2.0;
  EXPECT_TRUE(region.PointInSet(Vector1d{qmin + eps}));
  EXPECT_TRUE(region.PointInSet(Vector1d{qmax - eps}));
  EXPECT_FALSE(region.PointInSet(Vector1d{qmin - eps}));
  EXPECT_FALSE(region.PointInSet(Vector1d{qmax + eps}));
}

}  // namespace planning
}  // namespace drake
