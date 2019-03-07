#include <random>
#include <stdexcept>

#include "bot_core/pointcloud_t.hpp"
#include "lcm/lcm-cpp.hpp"

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/multibody/collision/element.h"
#include "drake/multibody/collision/model.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"
#include "drake/systems/estimators/dev/rotation.h"

// Note: Currently assumes that the tree has only the world plus a single body,
// with a single collision element, which is a box.  Hey, it's a start.
//
//  min_R,t,W sum_k | R*points.col(k) + t - vertices*W.col(k) |^2
//   subject to 0 <= W(i,j) <= 1, sum_j W(j,k)=1
//              W(i,k) = 0 or W(j,k) = 0 if i,j are not on the same face,
//              and an approximation of R'=R^{-1}, det(R)=1.
Eigen::Isometry3d PoseEstimation(const RigidBodyTree<double>& tree,
                                 const Eigen::Ref<Eigen::Matrix3Xd>& points) {
  // Extract collision element from first non-world body.
  DRAKE_DEMAND(tree.get_num_bodies() == 2);
  const auto& collision_element_ids =
      tree.get_body(1).get_collision_element_ids();
  DRAKE_DEMAND(collision_element_ids.size() == 1);
  const drake::multibody::collision::Element* collision_element =
      tree.FindCollisionElement(collision_element_ids[0]);
  DRAKE_DEMAND(collision_element->getShape() == DrakeShapes::BOX);

  // Extract box vertices.
  Eigen::Matrix3Xd vertices;
  collision_element->getGeometry().getPoints(vertices);
  std::cout << "Attempting to estimate the pose of a box w/ vertices:"
            << std::endl
            << vertices << std::endl;

  // Hard-code faces. Column i contains the indices of the vertices on the ith
  // face.
  // TODO(russt/SeanCurtis-TRI): Move into the DrakeShapes::Geometry API.
  Eigen::MatrixXi faces(4, 6);
  faces.col(0) << 1, 2, 5, 6;  // +X
  faces.col(1) << 0, 3, 4, 7;  // -X
  faces.col(2) << 0, 1, 2, 3;  // +Y
  faces.col(3) << 4, 5, 6, 7;  // -Y
  faces.col(4) << 0, 1, 6, 7;  // +Z
  faces.col(5) << 2, 3, 4, 5;  // -Z

  drake::solvers::MathematicalProgram prog;
  auto t = prog.NewContinuousVariables<3>("t");
  auto W = prog.NewContinuousVariables(vertices.cols(), points.cols(), "W");

  // Forall i,j, 0 <= W(i,j) <= 1.
  prog.AddBoundingBoxConstraint(0, 1, W);

  // Forall k, sum_j W(j,k) = 1.
  // Note: Adding constraints one at a time because W is not a column vector.
  for (int k = 0; k < points.cols(); k++) {
    prog.AddLinearEqualityConstraint(Eigen::MatrixXd::Ones(1, vertices.cols()),
                                     1, W.col(k));
  }

  enum RotationType {
    kTranslationOnly,
    kSdpRelaxationSpectrahedron,
    kSdpRelaxationPsdLift
  };

  const RotationType rotation_type = kTranslationOnly;
  //  const RotationType rotation_type = kSdpRelaxationSpectrahedron;
  drake::solvers::MatrixXDecisionVariable R;

  switch (rotation_type) {
    case kTranslationOnly: {
      // Doesn't use R.

      // Sum_k | points.col(k) + t - vertices*W.col(k) |^2
      Eigen::Matrix3Xd A(3, vertices.cols() + 3);
      A << Eigen::Matrix3d::Identity(), -vertices;
      for (int k = 0; k < points.cols(); k++) {
        prog.AddL2NormCost(A, points.col(k), {t, W.col(k)});
      }
    } break;
    case kSdpRelaxationSpectrahedron: {
      // Note: Without the integer constraints, this problem has a trivial
      // solution with R = 0, t = 0, and W = 0.125.

      R = drake::solvers::AddRotationMatrixSpectrahedralSdpRelaxation(&prog,
                                                                      "R");

      // min sum_k | R*points.col(k) + t - vertices*W.col(k) |^2
      //     aka sum_k | Ak * x |^2
      //
      // To formulate it as a linear objective (for SDP), we write
      // min sum_k sigma_k
      //   s.t. zk = Ak * x, sigma_k >= sqrt(z0^2 + ... z2^2)
      // This differs (by a sqrt) from the original objective, but should
      // be as good.
      // Note: If we really want the original objective, we can use the
      // rotated Lorentz cone
      //   sigma_k * dummy >= z0^2 + ... z2^2
      //   dummy = 1
      //
      // Note: Could have alternatively written the objective as
      //   x'Qx where Q = sum_k Ak'Ak, then used
      // min sigma
      //   s.t. z = sqrt(Q)*x, sigma >= sqrt(z0^2 + ... zn^2)
      // (this would add one big SOC constraint instead of many 3d constraints,
      // and would actually add more decision variables).

      auto Z = prog.NewContinuousVariables(3, points.cols(), "Z");
      auto sigma = prog.NewContinuousVariables(points.cols(), "sigma");

      Eigen::Matrix3Xd A(3, 9 + 3 + vertices.cols() + 3);
      A << Eigen::Matrix<double, 3, 9>::Zero(), Eigen::Matrix3d::Identity(),
          -vertices, -Eigen::Matrix3d::Identity();
      for (int k = 0; k < points.cols(); k++) {
        A(0, 0) = points(0, k);
        A(1, 1) = points(0, k);
        A(2, 2) = points(0, k);

        A(0, 3) = points(1, k);
        A(1, 4) = points(1, k);
        A(2, 5) = points(1, k);

        A(0, 6) = points(2, k);
        A(1, 7) = points(2, k);
        A(2, 8) = points(2, k);

        // z.col(k) = R*points.col(k) + t + vertices*W.col(k)
        prog.AddLinearEqualityConstraint(
            A, Eigen::Vector3d::Zero(),
            {R.col(0), R.col(1), R.col(2), t, W.col(k), Z.col(k)});

        // sigma(k) > sqrt(z(0,k)^2 + z(1,k)^2 + z(2,k)^2)
        prog.AddLorentzConeConstraint({sigma.segment<1>(k), Z.col(k)});
      }

      // min sum_k sigma_k
      prog.AddLinearCost(Eigen::VectorXd::Ones(points.cols()), sigma);
    } break;
    default:
      throw std::runtime_error("Unsupported rotation type.");
  }

  // W(i,k) = 0 or W(j,k) = 0 if i,j are not on the same face.
  if (rotation_type == kTranslationOnly) {  // Mosek doesn't support MISDP.  :(
    // Binary variables B(i,j) = 1 iff point j is on face i.
    auto B = prog.NewBinaryVariables(faces.cols(), points.cols(), "B");
    for (int k = 0; k < points.cols(); k++) {
      // Sum_i B(i,k) = 1 (each point must be assigned to exactly one face).
      prog.AddLinearEqualityConstraint(Eigen::RowVectorXd::Ones(faces.cols()),
                                       1, B.col(k));
    }

    // W(v,k) <= sum_j B(j,k) for all faces j that contain vertex v.
    Eigen::RowVectorXd A = Eigen::RowVectorXd::Ones(1 + faces.cols());
    A(0) = -1;
    for (int v = 0; v < vertices.cols(); v++) {
      for (int j = 0; j < faces.cols(); j++) {
        bool contains_vertex = false;
        for (int i = 0; i < faces.rows(); i++) {
          if (faces(i, j) == v) {
            contains_vertex = true;
          }
        }
        A(1 + j) = contains_vertex ? 1.0 : 0.0;
      }
      for (int k = 0; k < points.cols(); k++) {
        prog.AddLinearConstraint(A, 0, 1, {W.block<1, 1>(v, k), B.col(k)});
      }
    }
  }

  std::cout << "Beginning solve... ";
  const drake::solvers::MathematicalProgramResult result = Solve(prog);
  std::cout << "Finished." << std::endl;

  for (int i = 0; i < prog.num_vars(); ++i) {
    std::cout << prog.decision_variable(i).get_name() << " = "
              << result.GetSolution(prog.decision_variable(i)) << "\n";
  }

  std::cout << "Solver " << result.get_solver_id().name() << " result "
            << static_cast<int>(result.get_solution_result()) << std::endl;
  Eigen::Isometry3d T;
  T.translation() = result.GetSolution(t);
  if (rotation_type == kTranslationOnly) {
    T.linear() = Eigen::Matrix3d::Identity();
  } else {
    T.linear() = result.GetSolution(R);
  }
  return T;
}

int main(int argc, char** argv) {
  std::mt19937 generator(42);

  std::shared_ptr<lcm::LCM> lcm(new lcm::LCM());
  if (!lcm->good()) {
    throw std::runtime_error("LCM is not good");
  }

  RigidBodyTree<double> robot;
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      drake::FindResourceOrThrow("drake/multibody/models/box.urdf"),
      drake::multibody::joints::kQuaternion, &robot);
  Eigen::VectorXd q0(robot.get_num_positions());
  q0.setZero();
  q0(6) = 1;
  KinematicsCache<double> cache = robot.doKinematics(q0);
  printf("Set up robot with %d positions\n", robot.get_num_positions());

  const int kNumRays = 50;

  // Perform a raycast through the origin from in many random directions.
  Eigen::Matrix3Xd origins(3, kNumRays);
  Eigen::Matrix3Xd endpoints(3, kNumRays);
  Eigen::VectorXd distances(kNumRays);
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
  Eigen::Matrix3Xd points(3, kNumRays);
  int num_points = 0;
  for (int k = 0; k < kNumRays; k++) {
    if (distances(k) >= 0) {
      Eigen::Vector3d dir =
          endpoints.block<3, 1>(0, k) - origins.block<3, 1>(0, k);
      dir /= dir.norm();
      points.col(num_points++) = origins.block<3, 1>(0, k) + dir * distances(k);
    }
  }
  points.conservativeResize(3, num_points);

  // Rotate and translate the points by a random amount.
  std::normal_distribution<double> randn;
  Eigen::Vector3d origin(randn(generator), randn(generator), randn(generator));

  // Random quaternion (algorithm from Eigen::Quaternion::UnitRandom, but using
  // c++11 rands).
  std::uniform_real_distribution<> urand(0, 1);
  const double u1 = urand(generator), u2 = 2 * M_PI * urand(generator),
               u3 = 2 * M_PI * urand(generator);
  const double a = sqrt(1 - u1), b = sqrt(u1);
  Eigen::Quaternion<double> q(a * sin(u2), a * cos(u2), b * sin(u3),
                              b * cos(u3));

  Eigen::Translation3d T(origin);
  for (int k = 0; k < num_points; k++) {
    points.col(k) = T * points.col(k);
    //    points.col(k) = T * q._transformVector(points.col(k));
  }

  Eigen::Isometry3d pose_estimate = PoseEstimation(robot, points);

  Eigen::Quaternion<double> q_estimated(pose_estimate.linear());
  Eigen::Vector3d origin_estimated = pose_estimate.translation();

  std::cout << "original:  t=" << origin.transpose()
            << ", q=" << q.coeffs().transpose() << std::endl;
  std::cout << "estimated: t=" << origin_estimated.transpose()
            << ", q=" << q_estimated.coeffs().transpose() << std::endl;

  std::cout << "estimation error: " << std::endl
            << " trans = " << (origin_estimated - origin).transpose()
            << std::endl
            << " quat = " << (q_estimated.coeffs() - q.coeffs()).transpose()
            << std::endl;

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
