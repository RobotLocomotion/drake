#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/proximity_utilities.h"
#include "drake/geometry/proximity_engine.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"

/* @file Pins down how far apart the FCL and Coal backends are allowed to be.

 The callback unit tests that assert specific numbers (distance_to_point,
 distance_to_shape and penetration_as_point_pair) are value-parameterized and
 remain pinned to FCL; mechanically doubling them would assert the same
 Drake-side arithmetic twice without saying anything about the backends. What
 is actually worth knowing is where the two libraries agree and where they do
 not, so this test asks that question directly: it runs the same query on both
 backends and compares the answers.

 The answer has a sharp shape, and the tolerances below encode it:

   - Where Drake resolves the pair with its own analytic code -- every
     Sphere-X pair except Sphere-Ellipsoid, every X-HalfSpace pair, and
     Box-Box via distance_to_shape_touching -- the two backends agree to
     round-off. The backend supplies only the shape's dimensions, which are
     the same numbers either way.
   - Where the pair falls through to the library's GJK/EPA solver -- anything
     involving an Ellipsoid, and the non-sphere primitive pairs -- they differ
     by the amount two independent iterative solvers differ. Signed distances
     agree to ~1e-5 and penetration depths and witness points to ~1e-3.

 The contact *normal* is the exception, and it is worth stating plainly: for
 solver-resolved penetrating pairs the two backends can disagree by as much
 as 0.3 in the unit normal, which is roughly 17 degrees. That is not
 round-off. Where two shallowly overlapping smooth shapes are concerned the
 minimum-penetration direction is ill conditioned -- a small difference in
 where EPA terminates picks a different face of the Minkowski difference --
 and the libraries land on different answers. The bound asserted below is a
 measured observation, not an endorsement; flipping the default will change
 contact normals in simulations by this much for these shape pairs.

 None of this is a bug in either library, but all of it is the cost of the
 default flip, measured. It is also why the characterization tables in
 query_object.h have to be re-measured when the default changes rather than
 carried over.  */

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;
using std::make_unique;
using std::string;
using std::unique_ptr;
using std::unordered_map;
using std::vector;

constexpr double kInf = std::numeric_limits<double>::infinity();

/* Agreement to round-off, for pairs Drake resolves itself. */
constexpr double kExact = 1e-14;
/* Agreement between two independent GJK/EPA implementations. Each of these
 is the worst value observed across the shape pairs and poses below, rounded
 up: distances 1.3e-5, witness points 1.0e-3, depths 9.9e-4. */
constexpr double kSolverDistance = 2e-5;
constexpr double kSolverWitness = 2e-3;
/* See the normal discussion in the file comment. Observed worst: 3.0e-1. */
constexpr double kSolverNormal = 4e-1;

unique_ptr<Shape> MakeShape(const string& name) {
  if (name == "Sphere") return make_unique<Sphere>(0.1);
  if (name == "Box") return make_unique<Box>(0.15, 0.2, 0.17);
  if (name == "Capsule") return make_unique<Capsule>(0.08, 0.2);
  if (name == "Cylinder") return make_unique<Cylinder>(0.09, 0.21);
  if (name == "Ellipsoid") return make_unique<Ellipsoid>(0.12, 0.09, 0.15);
  if (name == "HalfSpace") return make_unique<HalfSpace>();
  throw std::runtime_error("unknown shape " + name);
}

/* Reports whether Drake has hand-written code for this pair, or whether it
 defers to the backend's solver. Mirrors shape_distance::RequiresFallback()
 and penetration_as_point_pair's narrowphase dispatch: Drake handles
 sphere-X for a limited set of X, and half spaces never reach the solver. */
bool DrakeResolvesItself(const string& a, const string& b) {
  if (a == "HalfSpace" || b == "HalfSpace") return true;
  if (a == "Box" && b == "Box") return true;
  const bool a_sphere = a == "Sphere";
  const bool b_sphere = b == "Sphere";
  if (!a_sphere && !b_sphere) return false;
  const string& other = a_sphere ? b : a;
  // Sphere-Ellipsoid and Sphere-Convex still go to the solver.
  return other != "Ellipsoid" && other != "Convex";
}

/* A spread of relative poses: clearly separated, nearly touching, and
 overlapping, with a non-trivial rotation so that no symmetry can hide a
 discrepancy. */
vector<RigidTransformd> RelativePoses() {
  const RollPitchYawd rpy(0.3, -0.5, 0.7);
  vector<RigidTransformd> out;
  for (double d : {0.5, 0.28, 0.22, 0.18}) {
    out.push_back(RigidTransformd(rpy, Vector3d{d, 0.02, -0.03}));
  }
  return out;
}

unique_ptr<ProximityEngine<double>> MakeEngine(CollisionBackend backend,
                                               const Shape& shape_A,
                                               const Shape& shape_B,
                                               GeometryId id_A,
                                               GeometryId id_B) {
  auto engine = make_unique<ProximityEngine<double>>(backend);
  const ProximityProperties props;
  engine->AddDynamicGeometry(shape_A, RigidTransformd{}, id_A, props);
  engine->AddDynamicGeometry(shape_B, RigidTransformd{}, id_B, props);
  return engine;
}

class BackendAgreementTest
    : public ::testing::TestWithParam<std::pair<string, string>> {};

TEST_P(BackendAgreementTest, QueriesAgree) {
  const auto& [name_A, name_B] = GetParam();
  SCOPED_TRACE(name_A + "-" + name_B);
  const bool exact = DrakeResolvesItself(name_A, name_B);
  const double distance_tol = exact ? kExact : kSolverDistance;
  const double witness_tol = exact ? kExact : kSolverWitness;
  const double normal_tol = exact ? kExact : kSolverNormal;

  const auto shape_A = MakeShape(name_A);
  const auto shape_B = MakeShape(name_B);
  const GeometryId id_A = GeometryId::get_new_id();
  const GeometryId id_B = GeometryId::get_new_id();

  auto fcl = MakeEngine(CollisionBackend::kFcl, *shape_A, *shape_B, id_A, id_B);
  auto coal =
      MakeEngine(CollisionBackend::kCoal, *shape_A, *shape_B, id_A, id_B);

  for (const RigidTransformd& X_AB : RelativePoses()) {
    SCOPED_TRACE(
        fmt::format("X_AB translation x = {}", X_AB.translation().x()));
    const unordered_map<GeometryId, RigidTransformd> X_WGs{
        {id_A, RigidTransformd{}}, {id_B, X_AB}};
    fcl->UpdateWorldPoses(X_WGs);
    coal->UpdateWorldPoses(X_WGs);

    // --- Pairwise signed distance ---
    {
      // Drake refuses signed distance for half space pairs other than
      // sphere-half space (see issue #10905). The backends must agree about
      // that too, so compare the refusals rather than skipping the pair.
      vector<SignedDistancePair<double>> f, c;
      string f_error, c_error;
      try {
        f = fcl->ComputeSignedDistancePairwiseClosestPoints(X_WGs, kInf);
      } catch (const std::exception& e) {
        f_error = e.what();
      }
      try {
        c = coal->ComputeSignedDistancePairwiseClosestPoints(X_WGs, kInf);
      } catch (const std::exception& e) {
        c_error = e.what();
      }
      EXPECT_EQ(f_error, c_error);
      // The backends must never disagree about *whether* there is a result.
      ASSERT_EQ(f.size(), c.size());
      if (f.size() == 1) {
        EXPECT_NEAR(f[0].distance, c[0].distance, distance_tol);
        EXPECT_TRUE(CompareMatrices(f[0].p_ACa, c[0].p_ACa, witness_tol));
        EXPECT_TRUE(CompareMatrices(f[0].p_BCb, c[0].p_BCb, witness_tol));
      }
    }

    // --- Point pair penetration ---
    {
      const auto f = fcl->ComputePointPairPenetration(X_WGs);
      const auto c = coal->ComputePointPairPenetration(X_WGs);
      ASSERT_EQ(f.size(), c.size());
      if (f.size() == 1) {
        // A sign error in ContactDepth() (D3) would show up here as a wildly
        // wrong value rather than a small one.
        EXPECT_NEAR(f[0].depth, c[0].depth, witness_tol);
        // Whatever they disagree about, both must report a unit normal.
        EXPECT_NEAR(f[0].nhat_BA_W.norm(), 1.0, kExact);
        EXPECT_NEAR(c[0].nhat_BA_W.norm(), 1.0, kExact);
        EXPECT_TRUE(
            CompareMatrices(f[0].nhat_BA_W, c[0].nhat_BA_W, normal_tol));
      }
    }

    // --- Collision candidates and the boolean query ---
    {
      EXPECT_EQ(fcl->HasCollisions(), coal->HasCollisions());
      EXPECT_EQ(fcl->FindCollisionCandidates(),
                coal->FindCollisionCandidates());
    }

    // --- Signed distance to a point ---
    {
      const Vector3d p_WQ{0.31, -0.11, 0.07};
      auto f = fcl->ComputeSignedDistanceToPoint(p_WQ, X_WGs, kInf);
      auto c = coal->ComputeSignedDistanceToPoint(p_WQ, X_WGs, kInf);
      ASSERT_EQ(f.size(), c.size());
      auto by_id = [](const auto& lhs, const auto& rhs) {
        return lhs.id_G < rhs.id_G;
      };
      std::sort(f.begin(), f.end(), by_id);
      std::sort(c.begin(), c.end(), by_id);
      for (size_t i = 0; i < f.size(); ++i) {
        // Point-to-shape is Drake's own code for every shape but Ellipsoid,
        // whose distance-to-point goes through the solver regardless of what
        // it is paired with.
        const bool point_exact =
            (f[i].id_G == id_A ? name_A : name_B) != "Ellipsoid";
        EXPECT_NEAR(f[i].distance, c[i].distance,
                    point_exact ? kExact : kSolverDistance);
      }
    }
  }
}

std::string PairName(
    const ::testing::TestParamInfo<std::pair<string, string>>& info) {
  return info.param.first + "_" + info.param.second;
}

INSTANTIATE_TEST_SUITE_P(
    ShapePairs, BackendAgreementTest,
    ::testing::Values(std::pair<string, string>{"Sphere", "Sphere"},
                      std::pair<string, string>{"Sphere", "Box"},
                      std::pair<string, string>{"Sphere", "Capsule"},
                      std::pair<string, string>{"Sphere", "Cylinder"},
                      std::pair<string, string>{"Sphere", "Ellipsoid"},
                      std::pair<string, string>{"Sphere", "HalfSpace"},
                      std::pair<string, string>{"Box", "Box"},
                      std::pair<string, string>{"Box", "Capsule"},
                      std::pair<string, string>{"Box", "Cylinder"},
                      std::pair<string, string>{"Box", "Ellipsoid"},
                      std::pair<string, string>{"Box", "HalfSpace"},
                      std::pair<string, string>{"Capsule", "Capsule"},
                      std::pair<string, string>{"Capsule", "Cylinder"},
                      std::pair<string, string>{"Capsule", "Ellipsoid"},
                      std::pair<string, string>{"Capsule", "HalfSpace"},
                      std::pair<string, string>{"Cylinder", "Cylinder"},
                      std::pair<string, string>{"Cylinder", "Ellipsoid"},
                      std::pair<string, string>{"Cylinder", "HalfSpace"},
                      std::pair<string, string>{"Ellipsoid", "Ellipsoid"},
                      std::pair<string, string>{"Ellipsoid", "HalfSpace"}),
    PairName);

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
