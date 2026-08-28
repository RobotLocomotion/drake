// The bounding-sphere radius property: for every supported shape class, at many
// random poses X_LG, every sampled surface point lies inside the reported
// sphere. A shape that picks up another shape's radius formula produces an
// unsound lambda with no other symptom, so the sweep covers the whole closed
// set of supported shapes and pins the throw-on-unsupported behaviour. Never
// loosen the tolerance to make a case pass.

#include "drake/planning/continuous_collision/bounding_sphere.h"

#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/fmt_eigen.h"
#include "drake/common/memory_file.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/in_memory_mesh.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/planning/continuous_collision/test/test_utilities.h"

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
using Eigen::Vector3d;
using test::Rng;
using test::Sampler;

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

/* For Convex and Mesh the "surface samples" are the convex-hull vertices
 themselves: they are the extreme points of the very hull object the proximity
 engine collides, so containing all of them is the whole claim. */
Sampler HullVertexSampler(
    const drake::geometry::PolygonSurfaceMesh<double>& hull) {
  return [&hull](Rng* rng) {
    const int v = test::UniformInt(rng, 0, hull.num_vertices() - 1);
    return Vector3d(hull.vertex(v));
  };
}

void CheckContainment(const Shape& shape, const Sampler& sampler,
                      const std::string& label, double translation_scale,
                      Rng* rng) {
  SCOPED_TRACE(label);
  for (int pose = 0; pose < kNumPoses; ++pose) {
    const RigidTransform<double> X_LG =
        test::RandomTransform(rng, translation_scale);
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

struct PrimitiveCase {
  std::string label;
  std::unique_ptr<Shape> shape;
  Sampler sampler;
};

/* Every supported primitive class, each at four sizes spanning the extremes the
 formulas have to survive: near-isotropic, needle-thin, plate-thin, and
 millimetre-scale. */
std::vector<PrimitiveCase> PrimitiveCases() {
  std::vector<PrimitiveCase> cases;
  const auto add = [&cases](std::string label, std::unique_ptr<Shape> shape,
                            Sampler sampler) {
    cases.push_back(
        PrimitiveCase{std::move(label), std::move(shape), std::move(sampler)});
  };
  for (const double r : {1e-4, 0.05, 1.0, 7.5}) {
    add(fmt::format("Sphere({})", r), std::make_unique<Sphere>(r),
        [r](Rng* rng) {
          return test::SampleSphere(rng, r);
        });
  }
  for (const Vector3d& s :
       {Vector3d(1.0, 1.0, 1.0), Vector3d(0.01, 2.0, 0.3),
        Vector3d(5.0, 0.002, 0.002), Vector3d(0.4, 0.7, 1.9)}) {
    add(fmt::format("Box({}, {}, {})", s.x(), s.y(), s.z()),
        std::make_unique<Box>(s.x(), s.y(), s.z()), [s](Rng* rng) {
          return test::SampleBox(rng, s);
        });
  }
  for (const Vector3d& c : {Vector3d(0.1, 1.0, 0), Vector3d(1.0, 0.01, 0),
                            Vector3d(0.001, 3.0, 0), Vector3d(0.5, 0.5, 0)}) {
    const double r = c.x();
    const double length = c.y();
    add(fmt::format("Capsule({}, {})", r, length),
        std::make_unique<Capsule>(r, length), [r, length](Rng* rng) {
          return test::SampleCapsule(rng, r, length);
        });
  }
  for (const Vector3d& c : {Vector3d(0.1, 1.0, 0), Vector3d(2.0, 0.01, 0),
                            Vector3d(0.002, 4.0, 0), Vector3d(0.5, 0.5, 0)}) {
    const double r = c.x();
    const double length = c.y();
    add(fmt::format("Cylinder({}, {})", r, length),
        std::make_unique<Cylinder>(r, length), [r, length](Rng* rng) {
          return test::SampleCylinder(rng, r, length);
        });
  }
  for (const Vector3d& e :
       {Vector3d(1.0, 1.0, 1.0), Vector3d(0.01, 0.5, 2.0),
        Vector3d(3.0, 0.001, 0.001), Vector3d(0.2, 0.9, 0.05)}) {
    add(fmt::format("Ellipsoid({}, {}, {})", e.x(), e.y(), e.z()),
        std::make_unique<Ellipsoid>(e.x(), e.y(), e.z()), [e](Rng* rng) {
          return test::SampleEllipsoid(rng, e);
        });
  }
  return cases;
}

GTEST_TEST(BoundingSphereTest, PrimitivesContainTheirSurface) {
  Rng rng(0x5eed0001);
  for (const PrimitiveCase& entry : PrimitiveCases()) {
    CheckContainment(*entry.shape, entry.sampler, entry.label, 2.0, &rng);
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
      v.col(i) = test::Uniform(rng, 0.2, 1.0) * test::RandomUnitVector(rng);
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
      v.col(col) = test::UniformVector(rng, -0.4, 0.4);
    }
    out.emplace_back("convex/redundant", v);
  }
  {  // Exactly planar (Drake documents this as non-degenerate).
    Eigen::Matrix3Xd v(3, 16);
    for (int i = 0; i < v.cols(); ++i) {
      v.col(i) = Vector3d(test::Uniform(rng, -1.0, 1.0),
                          test::Uniform(rng, -1.0, 1.0), 0.0);
    }
    out.emplace_back("convex/planar", v);
  }
  {  // Near-degenerate slab: 1 µm thick, 1 m wide.
    Eigen::Matrix3Xd v(3, 24);
    for (int i = 0; i < v.cols(); ++i) {
      v.col(i) =
          Vector3d(test::Uniform(rng, -1.0, 1.0), test::Uniform(rng, -1.0, 1.0),
                   test::Uniform(rng, -5e-7, 5e-7));
    }
    out.emplace_back("convex/thin-slab", v);
  }
  {  // Near-sliver: nearly one-dimensional.
    Eigen::Matrix3Xd v(3, 20);
    for (int i = 0; i < v.cols(); ++i) {
      v.col(i) = Vector3d(test::Uniform(rng, -2.0, 2.0),
                          test::Uniform(rng, -1e-5, 1e-5),
                          test::Uniform(rng, -1e-5, 1e-5));
    }
    out.emplace_back("convex/sliver", v);
  }
  {  // Tiny.
    Eigen::Matrix3Xd v(3, 20);
    for (int i = 0; i < v.cols(); ++i) {
      v.col(i) = 1e-4 * test::RandomUnitVector(rng);
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

/* A small nonconvex OBJ (an L-shaped prism) so the Mesh path exercises
 hull-vs-mesh semantics, not just a convex primitive in disguise. It is built in
 memory: nothing here needs a file on disk, and a write that silently failed
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
GTEST_TEST(BoundingSphereTest, ThrowsOnUnsupportedShapes) {
  const RigidTransform<double> X_LG = RigidTransform<double>::Identity();
  DRAKE_EXPECT_THROWS_MESSAGE(ComputeBoundingSphere(HalfSpace(), X_LG),
                              ".*HalfSpace.*");
  EXPECT_THROW(ComputeBoundingSphere(MeshcatCone(1.0, 0.5, 0.25), X_LG),
               std::exception);
}

}  // namespace
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
