#include "drake/planning/continuous_collision/benchmark/scenario_worlds.h"

#include <cmath>

#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/coulomb_friction.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/planning/robot_diagram_builder.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace benchmark {
namespace {

using drake::geometry::Box;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using drake::multibody::CoulombFriction;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::planning::RobotDiagram;
using drake::planning::RobotDiagramBuilder;
using Eigen::MatrixXd;
using Eigen::Vector3d;

CoulombFriction<double> Friction() {
  return CoulombFriction<double>(1.0, 1.0);
}

/// Adds one anchored box to the "environment" model instance.
void AddAnchoredBox(MultibodyPlant<double>* plant, const std::string& name,
                    const Vector3d& size, const RigidTransformd& X_WB) {
  if (!plant->HasModelInstanceNamed("environment")) {
    plant->AddModelInstance("environment");
  }
  const ModelInstanceIndex instance =
      plant->GetModelInstanceByName("environment");
  const RigidBody<double>& body =
      plant->AddRigidBody(name, instance,
                          SpatialInertia<double>::SolidBoxWithMass(
                              1.0, size.x(), size.y(), size.z()));
  plant->WeldFrames(plant->world_frame(), body.body_frame(), X_WB);
  plant->RegisterCollisionGeometry(body, RigidTransformd(),
                                   Box(size.x(), size.y(), size.z()),
                                   name + "_geom", Friction());
}

void AddTable(MultibodyPlant<double>* plant) {
  AddAnchoredBox(plant, "table", Vector3d(3.0, 3.0, 0.10),
                 RigidTransformd(Vector3d(0.0, 0.0, -0.05)));
}

/// The bookcase: two side panels, four horizontal boards (bottom, the two
/// bounding the reached-into bay, and top) and a back panel — seven anchored
/// boxes, all in the "environment" instance so they form one welded subgraph
/// with the world and with each other.
void AddShelf(MultibodyPlant<double>* plant, double s) {
  using S = ShelfGeometry;
  const double x_front = S::kFrontX + s;
  const double x_back = x_front + S::kDepth;
  const double bay_h = S::kBayHalfHeight + s;
  const double x_mid = 0.5 * (x_front + x_back);
  const double z_mid = 0.5 * (S::kBottomZ + S::kTopZ);
  const double height = S::kTopZ - S::kBottomZ;
  const double width = 2.0 * S::kHalfWidth;

  AddAnchoredBox(
      plant, "shelf_side_l", Vector3d(S::kDepth, S::kPanel, height),
      RigidTransformd(Vector3d(x_mid, S::kHalfWidth + 0.5 * S::kPanel, z_mid)));
  AddAnchoredBox(plant, "shelf_side_r", Vector3d(S::kDepth, S::kPanel, height),
                 RigidTransformd(
                     Vector3d(x_mid, -S::kHalfWidth - 0.5 * S::kPanel, z_mid)));
  AddAnchoredBox(plant, "shelf_board_bottom",
                 Vector3d(S::kDepth, width, S::kPanel),
                 RigidTransformd(Vector3d(x_mid, 0.0, S::kBottomZ)));
  AddAnchoredBox(plant, "shelf_board_low",
                 Vector3d(S::kDepth, width, S::kPanel),
                 RigidTransformd(Vector3d(
                     x_mid, 0.0, S::kBayCentreZ - bay_h - 0.5 * S::kPanel)));
  AddAnchoredBox(plant, "shelf_board_high",
                 Vector3d(S::kDepth, width, S::kPanel),
                 RigidTransformd(Vector3d(
                     x_mid, 0.0, S::kBayCentreZ + bay_h + 0.5 * S::kPanel)));
  AddAnchoredBox(plant, "shelf_board_top",
                 Vector3d(S::kDepth, width, S::kPanel),
                 RigidTransformd(Vector3d(x_mid, 0.0, S::kTopZ)));
  AddAnchoredBox(
      plant, "shelf_back", Vector3d(S::kPanel, width + 2.0 * S::kPanel, height),
      RigidTransformd(Vector3d(x_back + 0.5 * S::kPanel, 0.0, z_mid)));
}

}  // namespace

const char* const kIiwaUrl =
    "package://drake_models/iiwa_description/urdf/"
    "iiwa14_spheres_dense_collision.urdf";

std::shared_ptr<RobotDiagram<double>> MakeShelfWorld(double shelf_scale) {
  RobotDiagramBuilder<double> builder(0.0);
  MultibodyPlant<double>& plant = builder.plant();
  builder.parser().AddModelsFromUrl(kIiwaUrl);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"),
                   RigidTransformd());
  AddTable(&plant);
  AddShelf(&plant, shelf_scale);
  plant.Finalize();
  return std::shared_ptr<RobotDiagram<double>>(builder.Build());
}

std::shared_ptr<RobotDiagram<double>> MakeDualArmWorld(double base_separation) {
  RobotDiagramBuilder<double> builder(0.0);
  MultibodyPlant<double>& plant = builder.plant();
  builder.parser().SetAutoRenaming(true);
  const ModelInstanceIndex arm_a =
      builder.parser().AddModelsFromUrl(kIiwaUrl).at(0);
  const ModelInstanceIndex arm_b =
      builder.parser().AddModelsFromUrl(kIiwaUrl).at(0);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base", arm_a),
                   RigidTransformd());
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base", arm_b),
                   RigidTransformd(RotationMatrixd::MakeZRotation(M_PI),
                                   Vector3d(base_separation, 0.0, 0.0)));
  AddTable(&plant);
  plant.Finalize();
  return std::shared_ptr<RobotDiagram<double>>(builder.Build());
}

MatrixXd ShelfTrajectoryWaypoints() {
  MatrixXd w(7, 7);
  // clang-format off
  w << 0.0,  0.196552,  0.096805, -0.016522, -0.138992, -0.254084, 0.0,
       0.0, -0.219376,  0.213452,  0.715028,  0.250007, -0.270004, 0.0,
       0.0,  0.220295,  0.105212,  0.034537, -0.061201, -0.171787, 0.0,
       0.0, -1.546472, -1.471150, -0.931648, -1.577882, -1.679656, 0.0,
       0.0,  0.035924, -0.011122, -0.017085, -0.005269, -0.019359, 0.0,
       0.0,  0.740798,  0.465992,  0.214658,  0.321424,  0.426071, 0.0,
       0.0,  0.0,       0.0,       0.0,       0.0,       0.0,      0.0;
  // clang-format on
  return w;
}

std::vector<double> ShelfTrajectoryTimes() {
  return {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
}

MatrixXd DualArmTrajectoryWaypoints() {
  Eigen::VectorXd reach(14);
  // Arm A and arm B reach poses: both extend forward at ~0.49 m with a small
  // base yaw so the wrists pass each other offset in y and z.
  reach << 0.12, 0.0, 0.0, -1.40, 0.0, 1.10, 0.0, -0.12, 0.0, 0.0, -1.70, 0.0,
      0.80, 0.0;
  Eigen::VectorXd offset = Eigen::VectorXd::Zero(14);
  offset(0) = 0.10;
  offset(3) = 0.08;
  offset(7) = -0.10;
  offset(10) = 0.08;

  MatrixXd w(14, 5);
  w.col(0) = Eigen::VectorXd::Zero(14);
  w.col(1) = 0.5 * reach;
  w.col(2) = reach;
  w.col(3) = 0.5 * reach + offset;
  w.col(4) = Eigen::VectorXd::Zero(14);
  return w;
}

std::vector<double> DualArmTrajectoryTimes() {
  return {0.0, 1.0, 2.0, 3.0, 4.0};
}

}  // namespace benchmark
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
