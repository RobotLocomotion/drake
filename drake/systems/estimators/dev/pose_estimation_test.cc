#include <random>
#include <stdexcept>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/pointcloud_t.hpp>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/collision/element.h"
#include "drake/multibody/collision/model.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/mathematical_program.h"

// Note: Currently assumes that the tree has only the world plus a single body,
// with a single collision element, which is a box.  Hey, it's a start.
//
//  min_t,W sum_k | points.col(k) + t - vertices*W.col(k) |^2
//   subject to 0 <= W(i,j) <= 1, sum_j W(j,k)=1
//              W(i,k) = 0 or W(j,k) = 0 if i,j are not on the same face.
//
Eigen::Translation3d PoseEstimationTranslationOnly(
    const RigidBodyTree<double>& tree,
    const Eigen::Ref<Eigen::Matrix3Xd>& points) {
  // Extract collision element from first non-world body.
  DRAKE_DEMAND(tree.get_num_bodies() == 2);
  const auto& collision_element_ids =
      tree.get_body(1).get_collision_element_ids();
  DRAKE_DEMAND(collision_element_ids.size() == 1);
  const DrakeCollision::Element* collision_element =
      tree.FindCollisionElement(collision_element_ids[0]);
  DRAKE_DEMAND(collision_element->getShape() == DrakeShapes::BOX);

  // Extract box vertices.
  Eigen::Matrix3Xd vertices;
  collision_element->getGeometry().getPoints(vertices);
  std::cout << "Attempting to estimate the pose of a box w/ vertices:"
            << std::endl
            << vertices << std::endl;
  
  // Hard-code faces.
  // TODO(russt/SeanCurtis-TRI): Move into the DrakeShapes::Geometry API.
  Eigen::MatrixXi faces(4,6);
  faces.col(0) << 1,2,5,6;  // +X
  faces.col(1) << 0,3,4,7;  // -X
  faces.col(2) << 0,1,2,3;  // +Y
  faces.col(3) << 4,5,6,7;  // -Y
  faces.col(4) << 0,1,6,7;  // +Z
  faces.col(5) << 2,3,4,5;  // -Z

  drake::solvers::MathematicalProgram prog;
  auto t = prog.AddContinuousVariables<3>("t");
  auto W = prog.AddContinuousVariables(vertices.cols(), points.cols(), "W");

  // Forall i,j, 0 <= W(i,j) <= 1.
  prog.AddBoundingBoxConstraint(0, 1, {W});

  // Forall k, sum_j W(j,k) = 1.
  // Note: Adding constraints one at a time because W is not a column vector.
  for (int k=0; k < points.cols(); k++) {
    prog.AddLinearConstraint(Eigen::MatrixXd::Ones(1,vertices.cols()),
      drake::Vector1d::Ones(), drake::Vector1d::Ones(), {W.col(k)});
  }
  

  { // sum_k | points.col(k) + t - vertices*W.col(k) |^2
    Eigen::Matrix3Xd A(3, vertices.cols() + 3);
    A << Eigen::Matrix3d::Identity(), -vertices;
    for (int k = 0; k < points.cols(); k++) {
      prog.AddL2NormCost(A, points.col(k), {t, W.col(k)});
    }
  }

  { // W(i,k) = 0 or W(j,k) = 0 if i,j are not on the same face.
    Eigen::RowVectorXd A = Eigen::RowVectorXd::Ones(1+faces.cols());
    A(0) = -1;
    
    drake::solvers::DecisionVariableVectorX vars(1+faces.cols());
    
    // Binary variables B(i,j) = 1 iff point j is on face i.
    auto B = prog.AddBinaryVariables(faces.cols(),points.cols(),"B");
    for (int k = 0; k < points.cols(); k++) {
      // Sum_i B(i,k) = 1 (each point must be assigned to exactly one face).
      prog.AddLinearConstraint(Eigen::RowVectorXd::Ones(faces.cols()), drake::Vector1d::Ones(), drake::Vector1d::Ones(),{B.col(k)});

      // W(v,k) < sum_j B(j,k) for all faces j that contain vertex v.
      for (int v = 0; v < vertices.cols(); v++) {
        vars(0) = W(v,k);
        for (int j = 0; j < faces.cols(); j++) {
          vars(1+j) = B(j,k);
          bool contains_vertex = false;
          for (int i=0; i < faces.rows(); i++) {
            if (faces(i,j)==v) { contains_vertex = true; }
          }
          A(1+j) = contains_vertex ? 1.0 : 0.0;
        }
        prog.AddLinearConstraint(A,drake::Vector1d::Zero(),drake::Vector1d::Ones(),{vars});
      }
    }
  }

  std::cout << "Beginning solve... ";
  auto r = prog.Solve();
  std::string solver_name;
  int solver_result;
  prog.GetSolverResult(&solver_name, &solver_result);
  std::cout << "Finished. (" << solver_name << " exit code = " << static_cast<int>(r) << ")." << std::endl;
  prog.PrintSolution();
  return Eigen::Translation3d(GetSolution(t));
}

int main(int argc, char** argv) {
  using namespace Eigen;
  using namespace drake;

  std::mt19937 generator(42);

  std::shared_ptr<lcm::LCM> lcm(new lcm::LCM());
  if (!lcm->good()) {
    throw std::runtime_error("LCM is not good");
  }

  RigidBodyTree<double> robot;
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      GetDrakePath() + "/multibody/models/box.urdf",
      drake::multibody::joints::kQuaternion, &robot);
  VectorXd q0(robot.get_num_positions());
  q0.setZero();
  q0(6) = 1;
  KinematicsCache<double> cache = robot.doKinematics(q0);
  printf("Set up robot with %d positions\n", robot.get_num_positions());

  const int kNumRays = 50;

  // Perform a raycast through the origin from in many random directions.
  Matrix3Xd origins(3, kNumRays);
  Matrix3Xd endpoints(3, kNumRays);
  VectorXd distances(kNumRays);
  std::uniform_real_distribution<> rand(-100, 100);
  for (int k = 0; k < kNumRays; k++) {
    origins(0, k) = rand(generator);
    origins(1, k) = rand(generator);
    origins(2, k) = rand(generator);

    endpoints(0, k) = -origins(0, k);
    endpoints(1, k) = -origins(1, k);
    endpoints(2, k) = -origins(2, k);
  }
  robot.collisionRaycast(cache, origins, endpoints, distances, false);

  // Process the raycast results to create the point cloud.
  Matrix3Xd points(3, kNumRays);
  int num_points = 0;
  for (int k = 0; k < kNumRays; k++) {
    if (distances(k) >= 0) {
      Vector3d dir = endpoints.block<3, 1>(0, k) - origins.block<3, 1>(0, k);
      dir /= dir.norm();
      points.col(num_points++) = origins.block<3, 1>(0, k) + dir * distances(k);
    }
  }
  points.conservativeResize(3, num_points);

  // Translate the points by a random amount.
  std::normal_distribution<double> randn;
  Eigen::Vector3d origin(randn(generator), randn(generator), randn(generator));
  Eigen::Translation3d T(origin);
  for (int k = 0; k < num_points; k++) {
    points.col(k) = T * points.col(k);
  }

  Eigen::Translation3d pose_estimate =
      PoseEstimationTranslationOnly(robot, points);

  std::cout << "estimation error: "
            << (origin - pose_estimate.translation()).transpose() << std::endl;

  // Publish current point cloud to drake viewer.
  bot_core::pointcloud_t msg;
  msg.utime = 0;
  msg.n_channels = 0;
  msg.n_points = num_points;
  for (int k = 0; k < num_points; k++) {
    std::vector<float> pt = {static_cast<float>(points(0, k)),
                             static_cast<float>(points(1, k)),
                             static_cast<float>(points(2, k))};
    msg.points.push_back(pt);
  }
  lcm->publish("DRAKE_POINTCLOUD_FROM_URDF", &msg);
  lcm->handleTimeout(0);

  return 0;
}
