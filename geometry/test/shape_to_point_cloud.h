#pragma once

#include <vector>

#include "drake/common/random.h"
#include "drake/common/schema/stochastic.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/shape_specification.h"
#include "drake/geometry/shape_specification_convex_mesh.h"
#include "drake/math/random_rotation.h"

namespace drake {
namespace geometry {
namespace {

struct PointCloud {
  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector3d> normals;
};

}  // namespace

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
    cloud.points.emplace_back(r * (T * m.col(0)));
    cloud.normals.emplace_back(T.rotation() * m.col(0));
  }
  return cloud;
}

PointCloud SampleCapsuleSurface(const Capsule& capsule,
                                const math::RigidTransformd& T, int n) {
  PointCloud cloud;
  cloud.points.reserve(n);
  cloud.normals.reserve(n);

  const double r = capsule.radius();
  const double h_l = capsule.length() / 2;
  drake::RandomGenerator rgn(0);

  // Sample points in a sphere. Add half length to move each point to the
  // correct position
  const int n_sphere = n / 2;
  for (int i = 0; i < n_sphere; ++i) {
    Eigen::Matrix3d m =
        math::UniformlyRandomRotationMatrix<double>(&rgn).matrix();
    Eigen::Vector3d p = r * m.col(0);
    p.z() += p.z() > 0 ? h_l : -h_l;
    cloud.points.emplace_back(T * p);
    cloud.normals.emplace_back(T.rotation() * m.col(0));
  }

  // Now sample over the cylinder
  drake::schema::Uniform uniform_generator(-h_l, +h_l);
  drake::schema::Uniform angle_generator(0.0, 1.0);
  const int n_cyl = n - n_sphere;
  for (int i = 0; i < n_cyl; ++i) {
    const double theta = 2.0 * M_PI * angle_generator.Sample(&rgn);
    const double z = uniform_generator.Sample(&rgn);
    const double c = std::cos(theta), s = std::sin(theta);
    const Eigen::Vector3d normal(c, s, 0.0);
    const Eigen::Vector3d p = Eigen::Vector3d(r * c, r * s, z);
    cloud.points.emplace_back(T * p);
    cloud.normals.emplace_back(T.rotation() * normal);
  }

  return cloud;
}

PointCloud SampleCylinderSurface(const Cylinder& cylinder,
                                 const math::RigidTransformd& T, int n) {
  PointCloud cloud;
  cloud.points.reserve(n);
  cloud.normals.reserve(n);

  const double r = cylinder.radius();
  const double h_l = cylinder.length() / 2;
  drake::RandomGenerator rgn(0);
  drake::schema::Uniform uniform_generator(-h_l, +h_l);
  drake::schema::Uniform angle_generator(0.0, 1.0);

  // Sample points on top and bottom of the cylinder
  const int n_top_bottom = n / 2;
  for (int i = 0; i < n_top_bottom; ++i) {
    const double theta = 2.0 * M_PI * angle_generator.Sample(&rgn);
    const double z = uniform_generator.Sample(&rgn);
    const double c = std::cos(theta), s = std::sin(theta);
    const bool top_bottom = z > 0;
    const Eigen::Vector3d normal(0, 0, top_bottom ? 1.0 : -1.0);
    const double l = angle_generator.Sample(&rgn);
    const Eigen::Vector3d p(l * r * c, l * r * s, top_bottom ? h_l : -h_l);
    cloud.points.emplace_back(T * p);
    cloud.normals.emplace_back(T.rotation() * normal);
  }

  // Now sample over the cylinder
  const int n_cyl = n - n_top_bottom;
  for (int i = 0; i < n_cyl; ++i) {
    const double theta = 2.0 * M_PI * angle_generator.Sample(&rgn);
    const double z = uniform_generator.Sample(&rgn);
    const double c = std::cos(theta), s = std::sin(theta);
    const Eigen::Vector3d normal(c, s, 0.0);
    const Eigen::Vector3d p(r * c, r * s, z);
    cloud.points.emplace_back(T * p);
    cloud.normals.emplace_back(T.rotation() * normal);
  }

  return cloud;
}

PointCloud SampleEllipsoidSurface(const Ellipsoid& ellipsoid,
                                  const math::RigidTransformd& T, int n) {
  PointCloud cloud;
  cloud.points.reserve(n);
  cloud.normals.reserve(n);
  const double a = ellipsoid.a();
  const double b = ellipsoid.b();
  const double c = ellipsoid.c();
  drake::RandomGenerator rgn(0);
  for (int i = 0; i < n; ++i) {
    Eigen::Matrix3d m =
        math::UniformlyRandomRotationMatrix<double>(&rgn).matrix();
    const Eigen::Vector3d dir = m.col(0);
    Eigen::Vector3d p(a * dir.x(), b * dir.y(), c * dir.z());
    Eigen::Vector3d normal(2 * p.x() / (a * a), 2 * p.y() / (b * b),
                           2 * p.z() / (c * c));
    cloud.points.emplace_back(T * p);
    cloud.normals.emplace_back(T.rotation() * normal.normalized());
  }
  return cloud;
}

PointCloud SampleMeshSurface(const Mesh& mesh,
                             const math::RigidTransformd& T, int n) {
  PointCloud cloud;
  cloud.points.reserve(n);
  cloud.normals.reserve(n);
  const auto surface =
      ReadObjToTriangleSurfaceMesh(mesh.source(), mesh.scale3());
  drake::RandomGenerator rgn(1);
  std::uniform_int_distribution<int> tri_dist(0, surface.num_triangles() - 1);
  drake::schema::Uniform bary(0.0, 1.0);
  for (int i = 0; i < n; ++i) {
    const int f = tri_dist(rgn);
    const auto& tri = surface.element(f);
    const Eigen::Vector3d& v0 = surface.vertex(tri.vertex(0));
    const Eigen::Vector3d& v1 = surface.vertex(tri.vertex(1));
    const Eigen::Vector3d& v2 = surface.vertex(tri.vertex(2));
    double u = bary.Sample(&rgn);
    double v = bary.Sample(&rgn);
    if (u + v > 1) {
      u = 1 - u;
      v = 1 - v;
    }
    Eigen::Vector3d p = v0 + u * (v1 - v0) + v * (v2 - v0);
    const Eigen::Vector3d normal = surface.face_normal(f);
    cloud.points.emplace_back(T * p);
    cloud.normals.emplace_back(T.rotation() * normal);
  }
  return cloud;
}

PointCloud SampleConvexSurface(const Convex& convex,
                               const math::RigidTransformd& T, int n) {
  PointCloud cloud;
  cloud.points.reserve(n);
  cloud.normals.reserve(n);
  const auto surface =
      ReadObjToTriangleSurfaceMesh(convex.source(), convex.scale3());
  drake::RandomGenerator rgn(1);
  std::uniform_int_distribution<int> tri_dist(0, surface.num_triangles() - 1);
  drake::schema::Uniform bary(0.0, 1.0);
  for (int i = 0; i < n; ++i) {
    const int f = tri_dist(rgn);
    const auto& tri = surface.element(f);
    const Eigen::Vector3d& v0 = surface.vertex(tri.vertex(0));
    const Eigen::Vector3d& v1 = surface.vertex(tri.vertex(1));
    const Eigen::Vector3d& v2 = surface.vertex(tri.vertex(2));
    double u = bary.Sample(&rgn);
    double v = bary.Sample(&rgn);
    if (u + v > 1) {
      u = 1 - u;
      v = 1 - v;
    }
    Eigen::Vector3d p = v0 + u * (v1 - v0) + v * (v2 - v0);
    const Eigen::Vector3d normal = surface.face_normal(f);
    cloud.points.emplace_back(T * p);
    cloud.normals.emplace_back(T.rotation() * normal);
  }
  return cloud;
}


}  // namespace geometry
}  // namespace drake
