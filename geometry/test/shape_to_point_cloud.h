#include "drake/common/random.h"
#include "drake/common/schema/stochastic.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/random_rotation.h"

namespace drake {
namespace geometry {
namespace {

struct PointCloud {
  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector3d> normals;
};

PointCloud SampleBoxSurface(const Box& box, const math::RigidTransformd& T,
                            int n) {
  PointCloud box_cloud;
  box_cloud.points.reserve(n);
  box_cloud.normals.reserve(n);

  // Half dimensions
  const double hw = box.width() / 2;
  const double hd = box.depth() / 2;
  const double hh = box.height() / 2;

  // Create a random number generator.
  drake::RandomGenerator generator(0);
  drake::schema::Uniform uniform_x(-hw, hw);
  drake::schema::Uniform uniform_y(-hd, hd);
  drake::schema::Uniform uniform_z(-hh, hh);

  int k = n / 6;

  // Plane in +x direction
  for (int i = 0; i < k; ++i) {
    double y = uniform_y.Sample(&generator);
    double z = uniform_z.Sample(&generator);
    Eigen::Vector3d p(hw, y, z);
    Eigen::Vector3d normal(1., 0., 0.);
    box_cloud.points.push_back(T * p);
    box_cloud.normals.push_back(T.rotation() * normal);
  }

  // Plane in -x direction
  for (int i = 0; i < k; ++i) {
    double y = uniform_y.Sample(&generator);
    double z = uniform_z.Sample(&generator);
    Eigen::Vector3d p(-hw, y, z);
    Eigen::Vector3d normal(-1., 0., 0.);
    box_cloud.points.push_back(T * p);
    box_cloud.normals.push_back(T.rotation() * normal);
  }

  // Plane in +y direction
  for (int i = 0; i < k; ++i) {
    double x = uniform_x.Sample(&generator);
    double z = uniform_z.Sample(&generator);
    Eigen::Vector3d p(x, hd, z);
    Eigen::Vector3d normal(0., 1., 0.);
    box_cloud.points.push_back(T * p);
    box_cloud.normals.push_back(T.rotation() * normal);
  }

  // Plane in -y direction
  for (int i = 0; i < k; ++i) {
    double x = uniform_x.Sample(&generator);
    double z = uniform_z.Sample(&generator);
    Eigen::Vector3d p(x, -hd, z);
    Eigen::Vector3d normal(0., -1., 0.);
    box_cloud.points.push_back(T * p);
    box_cloud.normals.push_back(T.rotation() * normal);
  }

  // Plane in +z direction
  for (int i = 0; i < k; ++i) {
    double x = uniform_x.Sample(&generator);
    double y = uniform_y.Sample(&generator);
    Eigen::Vector3d p(x, y, hh);
    Eigen::Vector3d normal(0., 0., 1.);
    box_cloud.points.push_back(T * p);
    box_cloud.normals.push_back(T.rotation() * normal);
  }

  // Plane in -z direction
  for (int i = 0; i < k; ++i) {
    double x = uniform_x.Sample(&generator);
    double y = uniform_y.Sample(&generator);
    Eigen::Vector3d p(x, y, -hh);
    Eigen::Vector3d normal(0., 0., -1.);
    box_cloud.points.push_back(T * p);
    box_cloud.normals.push_back(T.rotation() * normal);
  }

  return box_cloud;
}

PointCloud SampleSphereSurface(const Sphere& sphere,
                               const math::RigidTransformd& T, int n) {
  PointCloud cloud;
  cloud.points.reserve(n);
  cloud.normals.reserve(n);

  drake::RandomGenerator rgn(0);
  const double r = sphere.radius();
  for (int i = 0; i < n; ++i) {
    Eigen::Matrix3d m =
        math::UniformlyRandomRotationMatrix<double>(&rgn).matrix();
    cloud.points.emplace_back(r * m.col(0));
    cloud.normals.emplace_back(m.col(0));
  }
  return cloud;
}

}  // namespace
}  // namespace geometry
}  // namespace drake