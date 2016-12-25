#include <random>
#include <stdexcept>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/pointcloud_t.hpp>

#include "drake/common/drake_path.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

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

  const int kNumRays = 500;

  // Perform a raycast through the origin from in many random directions.
  Matrix3Xd origins(3, kNumRays);
  Matrix3Xd endpoints(3, kNumRays);
  VectorXd distances(kNumRays);
  std::uniform_real_distribution<> rand(-100,100);
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
  Matrix<double, 3, Eigen::Dynamic> points(3,kNumRays);
  int num_points = 0;
  for (int k = 0; k < kNumRays; k++) {
    if (distances(k) >= 0) {
      Vector3d dir = endpoints.block<3, 1>(0, k) - origins.block<3, 1>(0, k);
      dir /= dir.norm();
      points.col(num_points++) = origins.block<3, 1>(0, k) + dir * distances(k);
    }
  }
  points.conservativeResize(3,num_points);
  
  // Translate the points by a random amount.
  std::normal_distribution<double> randn;
  Eigen::Vector3d origin(randn(generator), randn(generator), randn(generator));
  Eigen::Translation3d T(origin);
  for (int k=0; k < num_points; k++) {
    points.col(k) = T*points.col(k);
  }
  
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
