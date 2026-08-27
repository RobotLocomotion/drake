// The bounding-sphere radius property: for every supported shape class, at many
// random poses X_LG, every sampled surface point lies inside the reported
// sphere. A shape that picks up another shape's radius formula produces an
// unsound λ with no other symptom, so the sweep covers the whole closed set of
// supported shapes and pins the throw-on-unsupported behaviour. Never loosen
// the tolerance to make a case pass.

#include "drake/planning/continuous_collision/bounding_sphere.h"

#include <cmath>
#include <functional>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/fmt_eigen.h"
#include "drake/common/memory_file.h"
#include "drake/geometry/in_memory_mesh.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace {

using drake::geometry::Box;
using drake::geometry::Capsule;
using drake::geometry::Convex;
using drake::geometry::Cylinder;
using drake::geometry::Ellipsoid;
using drake::geometry::HalfSpace;
using drake::geometry::Mesh;
using drake::geometry::MeshcatCone;
using drake::geometry::Shape;
using drake::geometry::Sphere;
using drake::math::RigidTransform;
using drake::math::RotationMatrix;
using Eigen::Vector3d;

constexpr int kNumPoses = 100;
constexpr int kNumSurfaceSamples = 1000;
/* The containment claim is exact mathematics; this only absorbs the rounding of
 re-evaluating it. The slack is taken relative to the origin-centred radius
 R_g = ‖c_L‖ + ρ, which is the form the property is stated in: the test forms
 ‖X_LG·p − c_L‖ by cancelling two quantities of magnitude ‖t‖, so its absolute
 rounding error scales with ‖t‖ and not with ρ. Scaling the slack by ρ alone
 would make the test's own arithmetic, rather than the formulas under test,
 decide the outcome for a millimetre-scale shape parked a metre away. */
constexpr double kRelativeSlack = 1e-12;

using Rng = std::mt19937_64;

double Uniform(Rng* rng, double lo, double hi) {
  return std::uniform_real_distribution<double>(lo, hi)(*rng);
}

Vector3d RandomUnitVector(Rng* rng) {
  std::normal_distribution<double> normal(0.0, 1.0);
  Vector3d v;
  do {
    v = Vector3d(normal(*rng), normal(*rng), normal(*rng));
  } while (v.norm() < 1e-9);
  return v.normalized();
}

RotationMatrix<double> RandomRotation(Rng* rng) {
  std::normal_distribution<double> normal(0.0, 1.0);
  Eigen::Quaterniond q;
  do {
    q = Eigen::Quaterniond(normal(*rng), normal(*rng), normal(*rng),
                           normal(*rng));
  } while (q.norm() < 1e-9);
  q.normalize();
  return RotationMatrix<double>(q);
}

RigidTransform<double> RandomTransform(Rng* rng, double translation_scale) {
  return RigidTransform<double>(
      RandomRotation(rng),
      Vector3d(Uniform(rng, -translation_scale, translation_scale),
               Uniform(rng, -translation_scale, translation_scale),
               Uniform(rng, -translation_scale, translation_scale)));
}

/* Samples a point on the surface of the shape, expressed in its canonical
 geometry frame G. */
using Sampler = std::function<Vector3d(Rng*)>;

Sampler SphereSampler(double r) {
  return [r](Rng* rng) -> Vector3d {
    // The explicit return type materializes the Eigen product before the
    // lambda returns; without it the deduced type is an expression template
    // referencing the RandomUnitVector temporary, which dangles once the
    // std::function wrapper converts the result.
    return r * RandomUnitVector(rng);
  };
}

Sampler BoxSampler(double w, double d, double h) {
  const Vector3d half(0.5 * w, 0.5 * d, 0.5 * h);
  return [half](Rng* rng) {
    const int axis = std::uniform_int_distribution<int>(0, 2)(*rng);
    const double sign =
        std::uniform_int_distribution<int>(0, 1)(*rng) == 0 ? -1.0 : 1.0;
    Vector3d p(Uniform(rng, -half.x(), half.x()),
               Uniform(rng, -half.y(), half.y()),
               Uniform(rng, -half.z(), half.z()));
    p(axis) = sign * half(axis);
    return p;
  };
}

Sampler CapsuleSampler(double r, double length) {
  const double half = 0.5 * length;
  return [r, half](Rng* rng) {
    // Total area is split between the cylindrical barrel and the two caps;
    // exact area weighting is irrelevant here, but every region must be
    // sampled.
    if (std::uniform_int_distribution<int>(0, 1)(*rng) == 0) {
      const double phi = Uniform(rng, 0.0, 2.0 * M_PI);
      return Vector3d(r * std::cos(phi), r * std::sin(phi),
                      Uniform(rng, -half, half));
    }
    const Vector3d u = RandomUnitVector(rng);
    const double z_center = u.z() >= 0.0 ? half : -half;
    return Vector3d(r * u.x(), r * u.y(), z_center + r * u.z());
  };
}

Sampler CylinderSampler(double r, double length) {
  const double half = 0.5 * length;
  return [r, half](Rng* rng) {
    const double phi = Uniform(rng, 0.0, 2.0 * M_PI);
    if (std::uniform_int_distribution<int>(0, 1)(*rng) == 0) {
      return Vector3d(r * std::cos(phi), r * std::sin(phi),
                      Uniform(rng, -half, half));
    }
    // Cap disk: sqrt keeps the sample uniform in area, and hits the rim.
    const double rho = r * std::sqrt(Uniform(rng, 0.0, 1.0));
    const double z =
        std::uniform_int_distribution<int>(0, 1)(*rng) == 0 ? -half : half;
    return Vector3d(rho * std::cos(phi), rho * std::sin(phi), z);
  };
}

Sampler EllipsoidSampler(double a, double b, double c) {
  return [a, b, c](Rng* rng) {
    const Vector3d u = RandomUnitVector(rng);
    return Vector3d(a * u.x(), b * u.y(), c * u.z());
  };
}

/* For Convex and Mesh the "surface samples" are the convex-hull vertices
 themselves: they are the extreme points of the very hull object the proximity
 engine collides, so containing all of them is the whole claim. */
Sampler HullVertexSampler(
    const drake::geometry::PolygonSurfaceMesh<double>& hull) {
  return [&hull](Rng* rng) {
    const int v =
        std::uniform_int_distribution<int>(0, hull.num_vertices() - 1)(*rng);
    return Vector3d(hull.vertex(v));
  };
}

void CheckContainment(const Shape& shape, const Sampler& sampler,
                      const std::string& label, double translation_scale,
                      Rng* rng) {
  SCOPED_TRACE(label);
  for (int pose = 0; pose < kNumPoses; ++pose) {
    const RigidTransform<double> X_LG = RandomTransform(rng, translation_scale);
    const BoundingSphere sphere = ComputeBoundingSphere(shape, X_LG);
    ASSERT_TRUE(std::isfinite(sphere.radius)) << label;
    ASSERT_GE(sphere.radius, 0.0) << label;
    ASSERT_TRUE(sphere.center_L.allFinite()) << label;
    const double origin_radius = sphere.center_L.norm() + sphere.radius;
    const double limit = sphere.radius + kRelativeSlack * origin_radius;
    for (int i = 0; i < kNumSurfaceSamples; ++i) {
      const Vector3d p_G = sampler(rng);
      const double distance = (X_LG * p_G - sphere.center_L).norm();
      ASSERT_LE(distance, limit)
          << label << ": pose " << pose << ", sample " << i << ", radius "
          << sphere.radius << ", p_G "
          << fmt::format("{}", fmt_eigen(p_G.transpose()));
    }
    // The reach chain consumes ‖c_L‖ + ρ as an origin-centred radius; check
    // that relaxation too, since the displacement lemma depends on it directly.
    for (int i = 0; i < 32; ++i) {
      const Vector3d p_G = sampler(rng);
      ASSERT_LE((X_LG * p_G).norm(), origin_radius * (1.0 + kRelativeSlack))
          << label << " (origin-centred R_g)";
    }
  }
}

GTEST_TEST(BoundingSphereTest, SphereContainsSurface) {
  Rng rng(0x5eed0001);
  for (double r : {1e-4, 0.05, 1.0, 7.5}) {
    const Sphere shape(r);
    CheckContainment(shape, SphereSampler(r), fmt::format("Sphere({})", r), 2.0,
                     &rng);
  }
}

GTEST_TEST(BoundingSphereTest, BoxContainsSurface) {
  Rng rng(0x5eed0002);
  const std::vector<Vector3d> sizes{
      {1.0, 1.0, 1.0}, {0.01, 2.0, 0.3}, {5.0, 0.002, 0.002}, {0.4, 0.7, 1.9}};
  for (const Vector3d& s : sizes) {
    const Box shape(s.x(), s.y(), s.z());
    CheckContainment(shape, BoxSampler(s.x(), s.y(), s.z()),
                     fmt::format("Box({}, {}, {})", s.x(), s.y(), s.z()), 2.0,
                     &rng);
  }
}

GTEST_TEST(BoundingSphereTest, CapsuleContainsSurface) {
  Rng rng(0x5eed0003);
  const std::vector<std::pair<double, double>> params{
      {0.1, 1.0}, {1.0, 0.01}, {0.001, 3.0}, {0.5, 0.5}};
  for (const auto& [r, length] : params) {
    const Capsule shape(r, length);
    CheckContainment(shape, CapsuleSampler(r, length),
                     fmt::format("Capsule({}, {})", r, length), 2.0, &rng);
  }
}

GTEST_TEST(BoundingSphereTest, CylinderContainsSurface) {
  Rng rng(0x5eed0004);
  const std::vector<std::pair<double, double>> params{
      {0.1, 1.0}, {2.0, 0.01}, {0.002, 4.0}, {0.5, 0.5}};
  for (const auto& [r, length] : params) {
    const Cylinder shape(r, length);
    CheckContainment(shape, CylinderSampler(r, length),
                     fmt::format("Cylinder({}, {})", r, length), 2.0, &rng);
  }
}

GTEST_TEST(BoundingSphereTest, EllipsoidContainsSurface) {
  Rng rng(0x5eed0005);
  const std::vector<Vector3d> radii{
      {1.0, 1.0, 1.0}, {0.01, 0.5, 2.0}, {3.0, 0.001, 0.001}, {0.2, 0.9, 0.05}};
  for (const Vector3d& e : radii) {
    const Ellipsoid shape(e.x(), e.y(), e.z());
    CheckContainment(shape, EllipsoidSampler(e.x(), e.y(), e.z()),
                     fmt::format("Ellipsoid({}, {}, {})", e.x(), e.y(), e.z()),
                     2.0, &rng);
  }
}

/* Builds a variety of vertex sets: generic, redundant (interior points that do
 not survive the hull), near-degenerate (a very thin slab and a near-sliver),
 exactly planar, and tiny. */
std::vector<std::pair<std::string, Eigen::Matrix3Xd>> MakeVertexSets(Rng* rng) {
  std::vector<std::pair<std::string, Eigen::Matrix3Xd>> out;

  {  // Generic cloud on a ball.
    Eigen::Matrix3Xd v(3, 30);
    for (int i = 0; i < v.cols(); ++i) {
      v.col(i) = Uniform(rng, 0.2, 1.0) * RandomUnitVector(rng);
    }
    out.emplace_back("convex/generic", v);
  }
  {  // Cube corners plus many redundant interior points.
    Eigen::Matrix3Xd v(3, 8 + 40);
    int col = 0;
    for (int sx : {-1, 1}) {
      for (int sy : {-1, 1}) {
        for (int sz : {-1, 1}) {
          v.col(col++) = Vector3d(0.5 * sx, 0.5 * sy, 0.5 * sz);
        }
      }
    }
    for (; col < v.cols(); ++col) {
      v.col(col) = Vector3d(Uniform(rng, -0.4, 0.4), Uniform(rng, -0.4, 0.4),
                            Uniform(rng, -0.4, 0.4));
    }
    out.emplace_back("convex/redundant", v);
  }
  {  // Exactly planar (Drake documents this as non-degenerate).
    Eigen::Matrix3Xd v(3, 16);
    for (int i = 0; i < v.cols(); ++i) {
      v.col(i) =
          Vector3d(Uniform(rng, -1.0, 1.0), Uniform(rng, -1.0, 1.0), 0.0);
    }
    out.emplace_back("convex/planar", v);
  }
  {  // Near-degenerate slab: 1 µm thick, 1 m wide.
    Eigen::Matrix3Xd v(3, 24);
    for (int i = 0; i < v.cols(); ++i) {
      v.col(i) = Vector3d(Uniform(rng, -1.0, 1.0), Uniform(rng, -1.0, 1.0),
                          Uniform(rng, -5e-7, 5e-7));
    }
    out.emplace_back("convex/thin-slab", v);
  }
  {  // Near-sliver: nearly one-dimensional.
    Eigen::Matrix3Xd v(3, 20);
    for (int i = 0; i < v.cols(); ++i) {
      v.col(i) = Vector3d(Uniform(rng, -2.0, 2.0), Uniform(rng, -1e-5, 1e-5),
                          Uniform(rng, -1e-5, 1e-5));
    }
    out.emplace_back("convex/sliver", v);
  }
  {  // Tiny.
    Eigen::Matrix3Xd v(3, 20);
    for (int i = 0; i < v.cols(); ++i) {
      v.col(i) = 1e-4 * RandomUnitVector(rng);
    }
    out.emplace_back("convex/tiny", v);
  }
  return out;
}

GTEST_TEST(BoundingSphereTest, ConvexContainsHullVertices) {
  Rng rng(0x5eed0006);
  int checked = 0;
  for (const auto& [name, vertices] : MakeVertexSets(&rng)) {
    for (const Vector3d& scale3 :
         {Vector3d(1.0, 1.0, 1.0), Vector3d(2.0, 0.5, 1.3)}) {
      const Convex shape(vertices, name, scale3);
      const drake::geometry::PolygonSurfaceMesh<double>* hull = nullptr;
      try {
        hull = &shape.GetConvexHull();
      } catch (const std::exception& e) {
        // Drake rejects hulls it considers degenerate; the checker inherits
        // that decision, and the radius claims nothing about a shape the
        // proximity engine cannot build either.
        GTEST_LOG_(INFO) << name << ": Drake refused the hull: " << e.what();
        continue;
      }
      ASSERT_GT(hull->num_vertices(), 0) << name;
      CheckContainment(shape, HullVertexSampler(*hull),
                       fmt::format("{} scale3=({}, {}, {})", name, scale3.x(),
                                   scale3.y(), scale3.z()),
                       1.0, &rng);
      ++checked;
    }
  }
  EXPECT_GE(checked, 4) << "too few Convex vertex sets survived hull "
                           "construction to make this test meaningful";
}

/* Builds a small nonconvex OBJ (an L-shaped prism) so the Mesh path exercises
 hull-vs-mesh semantics, not just a convex primitive in disguise. It is built
 in memory: nothing here needs a file on disk, and a write that silently failed
 would turn this case into a vacuous pass. */
geometry::InMemoryMesh LShapedObj() {
  std::ostringstream out;
  // Six-vertex L profile in the z = ±0.25 planes.
  const std::vector<std::pair<double, double>> profile{
      {0.0, 0.0}, {1.0, 0.0}, {1.0, 0.3}, {0.3, 0.3}, {0.3, 1.2}, {0.0, 1.2}};
  for (double z : {-0.25, 0.25}) {
    for (const auto& [x, y] : profile) {
      out << "v " << x << " " << y << " " << z << "\n";
    }
  }
  // Two end caps as fans plus the side quads (triangulated); winding does not
  // matter for the convex hull.
  for (int base : {1, 7}) {
    for (int i = 1; i + 1 < 6; ++i) {
      out << "f " << base << " " << base + i << " " << base + i + 1 << "\n";
    }
  }
  for (int i = 0; i < 6; ++i) {
    const int a = 1 + i;
    const int b = 1 + (i + 1) % 6;
    out << "f " << a << " " << b << " " << b + 6 << "\n";
    out << "f " << a << " " << b + 6 << " " << a + 6 << "\n";
  }
  return geometry::InMemoryMesh{
      MemoryFile(out.str(), ".obj", "ccd_l_prism.obj")};
}

GTEST_TEST(BoundingSphereTest, MeshContainsHullVertices) {
  Rng rng(0x5eed0007);
  for (const Vector3d& scale3 :
       {Vector3d(1.0, 1.0, 1.0), Vector3d(0.4, 1.7, 1.0)}) {
    const Mesh shape(LShapedObj(), scale3);
    const auto& hull = shape.GetConvexHull();
    ASSERT_GT(hull.num_vertices(), 3);
    CheckContainment(shape, HullVertexSampler(hull),
                     fmt::format("Mesh scale3=({}, {}, {})", scale3.x(),
                                 scale3.y(), scale3.z()),
                     1.0, &rng);
  }
}

/* A shape that is not on the supported list must throw, never silently inherit
 some other shape's formula. */
GTEST_TEST(BoundingSphereTest, ThrowsOnHalfSpace) {
  const HalfSpace shape;
  const RigidTransform<double> X_LG = RigidTransform<double>::Identity();
  EXPECT_THROW(ComputeBoundingSphere(shape, X_LG), std::exception);
  try {
    ComputeBoundingSphere(shape, X_LG);
    GTEST_FAIL() << "expected a throw";
  } catch (const std::exception& e) {
    const std::string what = e.what();
    EXPECT_NE(what.find("HalfSpace"), std::string::npos) << what;
  }
}

GTEST_TEST(BoundingSphereTest, ThrowsOnUnsupportedShape) {
  const MeshcatCone shape(1.0, 0.5, 0.25);
  const RigidTransform<double> X_LG = RigidTransform<double>::Identity();
  EXPECT_THROW(ComputeBoundingSphere(shape, X_LG), std::exception);
}

}  // namespace
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
