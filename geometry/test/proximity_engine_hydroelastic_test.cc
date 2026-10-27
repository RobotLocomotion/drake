#include <memory>
#include <ranges>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_state.h"
#include "drake/geometry/proximity/hydroelastic_calculator.h"
#include "drake/geometry/proximity_engine.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

class ProximityEngineTester {
 public:
  ProximityEngineTester() = delete;

  template <typename T>
  static HydroelasticType hydroelastic_type(GeometryId id,
                                            const ProximityEngine<T>& engine) {
    return engine.hydroelastic_geometries().hydroelastic_type(id);
  }
};

namespace {

using Eigen::Vector3d;
using math::RigidTransform;
using math::RigidTransformd;
using std::make_unique;
using std::unordered_map;
using std::vector;

class ProximityEngineHydroelasticTest : public ::testing::Test {
 protected:
  using Tester = ProximityEngineTester;
  using V3 = Vector3d;

  GeometryId AddDynamic(const Shape& shape,
                        const std::optional<Vector3d>& p_WG = std::nullopt,
                        const ProximityProperties& props = {}) {
    const GeometryId id = GeometryId::get_new_id();
    const RigidTransformd X_WG(p_WG.value_or(Vector3d::Zero()));
    X_WGs_.insert({id, X_WG});
    engine_.AddDynamicGeometry(shape, X_WG, id, props);
    return id;
  }

  GeometryId AddAnchored(const Shape& shape,
                         const std::optional<Vector3d>& p_WG = std::nullopt,
                         const ProximityProperties& props = {}) {
    const GeometryId id = GeometryId::get_new_id();
    const RigidTransformd X_WG(p_WG.value_or(Vector3d::Zero()));
    X_WGs_.insert({id, X_WG});
    engine_.AddAnchoredGeometry(shape, X_WG, id, props);
    return id;
  }

  ProximityEngine<double> engine_;
  unordered_map<GeometryId, RigidTransformd> X_WGs_;
};

// "Processing" hydroelastic geometry is mostly a case of invoking a method
// on hydroelastic::Geometries. This test merely confirms that properties for
// the shapes reach hydroelastic::Geometries. It exploits the understanding that
// there are two patterns for how shapes are registered: one for Convex/Mesh
// and one for everything else. As such, it uses a Mesh and a Sphere to test the
// two patterns (inferring success for all other shapes).
TEST_F(ProximityEngineHydroelasticTest, ProcessHydroelasticProperties) {
  // All of the geometries will have a scale comparable to edge_length, so that
  // the mesh creation is as cheap as possible. The exception is Mesh
  // geometry since we have no re-meshing.
  const double edge_length = 0.5;
  const double E = 1e8;  // Elastic modulus.
  ProximityProperties compliant_properties;
  AddCompliantHydroelasticProperties(edge_length, E, &compliant_properties);
  ProximityProperties rigid_properties;
  AddRigidHydroelasticProperties(edge_length, &rigid_properties);

  using enum HydroelasticType;

  // Case: primitive shape.
  const GeometryId sphere_id =
      AddDynamic(Sphere{edge_length}, {}, compliant_properties);
  EXPECT_EQ(Tester::hydroelastic_type(sphere_id, engine_), kCompliant);

  // Case: mesh-type shape.
  const GeometryId rigid_mesh_id =
      AddDynamic(Mesh(drake::FindResourceOrThrow(
                     "drake/geometry/test/non_convex_mesh.obj")),
                 {}, rigid_properties);
  EXPECT_EQ(Tester::hydroelastic_type(rigid_mesh_id, engine_), kRigid);
}

// Test a combination that used to throw an exception: a vtk volume mesh with
// no hydroelastic type declared.
TEST_F(ProximityEngineHydroelasticTest, ProcessVtkUndefHydro) {
  const std::string vtk_path =
      FindResourceOrThrow("drake/geometry/test/non_convex_mesh.vtk");
  using enum HydroelasticType;

  // Case: Mesh.
  const GeometryId mesh_id = AddDynamic(Mesh(vtk_path));
  EXPECT_EQ(Tester::hydroelastic_type(mesh_id, engine_), kUndefined);

  // Case: Convex.
  const GeometryId convex_id = AddDynamic(Convex(vtk_path));
  EXPECT_EQ(Tester::hydroelastic_type(convex_id, engine_), kUndefined);
}

// Tests replacing the proximity properties for a given geometry.
TEST_F(ProximityEngineHydroelasticTest, ReplaceProperties) {
  // Some quick aliases to make the tests more compact.
  using enum HydroelasticType;

  const double radius = 0.5;
  InternalGeometry sphere(SourceId::get_new_id(), make_unique<Sphere>(radius),
                          FrameId::get_new_id(), GeometryId::get_new_id(),
                          "sphere", RigidTransformd());

  // Note: The order of these tests matter; one builds on the next. Re-ordering
  // *may* break the test.

  // Case: throws when the id doesn't refer to a valid geometry.
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine_.UpdateRepresentationForNewProperties(sphere, {}),
      "The proximity engine does not contain a geometry with the id \\d+; its "
      "properties cannot be updated");

  // Case: The new and old properties have no hydroelastic declarations, however
  // it mindlessly attempts to update the hydroelastic representation.
  {
    ProximityProperties props;
    props.AddProperty("foo", "bar", 1.0);
    engine_.AddDynamicGeometry(sphere.shape(), {}, sphere.id(), props);
    ASSERT_EQ(Tester::hydroelastic_type(sphere.id(), engine_), kUndefined);
    DRAKE_EXPECT_NO_THROW(
        engine_.UpdateRepresentationForNewProperties(sphere, {}));
    EXPECT_EQ(Tester::hydroelastic_type(sphere.id(), engine_), kUndefined);
  }

  // Case: The new set has hydroelastic properties, the old does not; change
  // required.
  {
    ProximityProperties props;
    // Pick a characteristic length sufficiently large that we create the
    // coarsest, cheapest mesh possible.
    ASSERT_EQ(Tester::hydroelastic_type(sphere.id(), engine_), kUndefined);
    AddRigidHydroelasticProperties(3 * radius, &props);
    DRAKE_EXPECT_NO_THROW(
        engine_.UpdateRepresentationForNewProperties(sphere, props));
    EXPECT_EQ(Tester::hydroelastic_type(sphere.id(), engine_), kRigid);
  }

  // Case: The new set does *not* have hydroelastic properties, the old does;
  // this should remove the hydroelastic representation.
  {
    ASSERT_EQ(Tester::hydroelastic_type(sphere.id(), engine_), kRigid);
    DRAKE_EXPECT_NO_THROW(engine_.UpdateRepresentationForNewProperties(
        sphere, ProximityProperties()));
    EXPECT_EQ(Tester::hydroelastic_type(sphere.id(), engine_), kUndefined);
  }

  // Create a baseline property set that requests a compliant hydroelastic
  // representation, but is not necessarily sufficient to define one.
  ProximityProperties hydro_trigger;
  hydro_trigger.AddProperty(kHydroGroup, kComplianceType, kCompliant);

  // Case: New properties request hydroelastic, but they are incomplete and
  // efforts to assign those properties throw.
  {
    ProximityProperties bad_props_no_elasticity(hydro_trigger);
    bad_props_no_elasticity.AddProperty(kHydroGroup, kRezHint, 1.25);
    DRAKE_EXPECT_THROWS_MESSAGE(
        engine_.UpdateRepresentationForNewProperties(sphere,
                                                     bad_props_no_elasticity),
        "Cannot create compliant Sphere; missing the .+ property");

    ProximityProperties bad_props_no_length(hydro_trigger);
    bad_props_no_length.AddProperty(kHydroGroup, kElastic, 5e8);
    DRAKE_EXPECT_THROWS_MESSAGE(
        engine_.UpdateRepresentationForNewProperties(sphere,
                                                     bad_props_no_length),
        "Cannot create compliant Sphere; missing the .+ property");
  }
}

/* ComputeContactSurfaces() responsibilities:
  1. Empty engine produces no results.
  2. Collision result for dynamic-dynamic pair.
  3. Collision result for anchored-dynamic pair.
  4. No collision result for anchored-anchored pair.
  5. Respect representation format.
  6. Results are always ordered.
  7. Incompatible geometries trigger a throw.
  8. Respect collision filter.
  9. AutoDiff derivatives pass through successfully.
 10. It should also allow Callback exceptions to propagate through; we won't
     test this directly -- Drake standard practices and the clear absence of a
     catch block is sufficient evidence

  Note: the sequence of this test is important -- state accumulates and each
  assertion is reliant on the successful execution of the previous assertion.

         ○○○     ●●●     □□□     ···
       ○     ○ ●     ● □     □ ·     ·     - Prefixes A, D indicated anchored,
     ○   A1  ● ○ D2  □ ●  D3 · □  D5   ·     dynamic.
     ○       ● ○     □ ●     · □       ·   - Suffixes indicate creation order.
       ○ ▲▲▲ ○ ●     ● □     □ ·     ·     - Intersecting pairs: (A1, A4),
       ▲ ○○○ ▲   ●●●     □□□     ···         (A1, D2), (D2, D3), (D3, D5).
     ▲         ▲                           - D5 (labeled "bad" below) has no
     ▲   A4    ▲                             hydro properties and will trigger
       ▲     ▲                               an exception.
         ▲▲▲
  */
TEST_F(ProximityEngineHydroelasticTest, ComputeContactSurfaces) {
  auto eval_dut = [this](HydroelasticContactRepresentation rep) {
    engine_.UpdateWorldPoses(X_WGs_);
    return engine_.ComputeContactSurfaces(rep, X_WGs_);
  };

  using IdPairs = std::vector<std::pair<GeometryId, GeometryId>>;
  auto extract_ids = [](const vector<ContactSurface<double>>& results) {
    auto ids = results |
               std::views::transform([](const ContactSurface<double>& result) {
                 return std::make_pair(result.id_M(), result.id_N());
               });
    return IdPairs(ids.begin(), ids.end());
  };

  using enum HydroelasticContactRepresentation;

  // (1) - empty engine produces no results.
  ASSERT_TRUE(eval_dut(kTriangle).empty());

  const Sphere sphere(0.5);
  // Distance between sphere centers to guarantee collision.
  const double d = sphere.radius() * 2 * 0.9;

  ProximityProperties props;
  AddCompliantHydroelasticProperties(0.5, 1e-8, &props);
  const GeometryId id_A1 = AddAnchored(sphere, V3{d, 0, 0}, props);
  const GeometryId id_D2 = AddDynamic(sphere, V3{2 * d, 0, 0}, props);
  const GeometryId id_D3 = AddDynamic(sphere, V3{3 * d, 0, 0}, props);
  // We don't need id_A4; it should appear in no results. Its presence would
  // indicate an error.
  AddAnchored(sphere, V3{d, -d, 0}, props);

  // (2)-(4).
  const vector<ContactSurface<double>> results_tri = eval_dut(kTriangle);
  // We get the (3) A-D and (2) D-D result but not an (4) A-A result.
  ASSERT_EQ(extract_ids(results_tri),
            IdPairs({{id_A1, id_D2}, {id_D2, id_D3}}));

  // (5) - consistent ordering in results.
  const vector<ContactSurface<double>> results_poly = eval_dut(kPolygon);
  ASSERT_EQ(extract_ids(results_poly), extract_ids(results_tri));

  // (6) - representation respected.
  for (const auto& result : results_poly) {
    EXPECT_FALSE(result.is_triangle());
  }
  for (const auto& result : results_tri) {
    EXPECT_TRUE(result.is_triangle());
  }

  // (7) - incompatible geometries trigger a throw; D5 is the "bad" geometry.
  const GeometryId id_bad =
      AddDynamic(sphere, V3{4 * d, 0, 0});  // No hydro props.
  EXPECT_THROW(eval_dut(kTriangle), std::logic_error);

  // (8) - filters are respected.
  engine_.collision_filter().Apply(
      CollisionFilterDeclaration().ExcludeWithin(GeometrySet()),
      [id_D3, id_bad](const GeometrySet&, CollisionFilterScope) {
        return std::unordered_set<GeometryId>{{id_D3, id_bad}};
      });
  ASSERT_EQ(eval_dut(kTriangle).size(), 2);

  // (9) - AutoDiff derivatives pass through successfully.
  const auto ad_engine = engine_.ToScalarType<AutoDiffXd>();
  unordered_map<GeometryId, RigidTransform<AutoDiffXd>> X_WGs_ad;
  for (const auto& [id, X_WG] : X_WGs_) {
    X_WGs_ad[id] = X_WG.cast<AutoDiffXd>();
  }
  // Seed id_D2's translation with derivatives.
  X_WGs_ad[id_D2] = RigidTransform<AutoDiffXd>{
      math::InitializeAutoDiff(X_WGs_.at(id_D2).translation())};

  ad_engine->UpdateWorldPoses(X_WGs_ad);
  const vector<ContactSurface<AutoDiffXd>> ad_results =
      ad_engine->ComputeContactSurfaces(kTriangle, X_WGs_ad);
  // Still get two results.
  ASSERT_EQ(ad_results.size(), 2);
  // Confirm derivatives on the second result -- not empty and not zero.
  const auto& result = ad_results[1];
  const auto& derivs = result.total_area().derivatives();
  ASSERT_EQ(derivs.size(), 3);
  EXPECT_FALSE(derivs.isZero());
}

/* ComputeContactSurfacesWithFallback() responsibilities:
  1. Empty engine produces no results.
  2. Collision result for dynamic-dynamic pair.
  3. Collision result for anchored-dynamic pair.
  4. No collision result for anchored-anchored pair.
  5. Respect representation format.
  6. Results are always ordered.
  7. No contact surface implies point pair penetration.
  8. Respect collision filter.
  9. AutoDiff derivatives pass through successfully.
 10. It should also allow Callback exceptions to propagate through; we won't
     test this directly -- Drake standard practices and the clear absence of a
     catch block is sufficient evidence

  Note: the sequence of this test is important -- state accumulates and each
  assertion is reliant on the successful execution of the previous assertion.

         ○○○     ●●●     □□□     ···
       ○     ○ ●     ● □     □ ·     ·     - Prefixes A, D indicated anchored,
     ○   A1  ● ○ D2  □ ●  D3 · □  D5   ·     dynamic.
     ○       ● ○     □ ●     · □       ·   - Suffixes indicate creation order.
       ○ ▲▲▲ ○ ●     ● □     □ ·     ·     - Intersecting pairs: (A1, A4),
       ▲ ○○○ ▲   ●●●     □□□     ···         (A1, D2), (D2, D3), (D3, D5).
     ▲         ▲                           - D5 has no hydro properties and will
     ▲   A4    ▲                             lead to a point-pair contact.
       ▲     ▲
         ▲▲▲
  */
TEST_F(ProximityEngineHydroelasticTest, ComputeContactSurfacesWithFallback) {
  vector<ContactSurface<double>> surfaces;
  vector<PenetrationAsPointPair<double>> points;
  auto eval_dut = [this, &points,
                   &surfaces](HydroelasticContactRepresentation rep) {
    surfaces.clear();
    points.clear();
    engine_.UpdateWorldPoses(X_WGs_);
    return engine_.ComputeContactSurfacesWithFallback(rep, X_WGs_, &surfaces,
                                                      &points);
  };

  using IdPairs = std::vector<std::pair<GeometryId, GeometryId>>;
  auto surface_ids = [](const vector<ContactSurface<double>>& results) {
    auto ids = results |
               std::views::transform([](const ContactSurface<double>& result) {
                 return std::make_pair(result.id_M(), result.id_N());
               });
    return IdPairs(ids.begin(), ids.end());
  };

  auto point_ids = [](const vector<PenetrationAsPointPair<double>>& results) {
    auto ids = results | std::views::transform(
                             [](const PenetrationAsPointPair<double>& result) {
                               return std::make_pair(result.id_A, result.id_B);
                             });
    return IdPairs(ids.begin(), ids.end());
  };

  using enum HydroelasticContactRepresentation;

  // (1) - empty engine produces no results.
  eval_dut(kTriangle);
  ASSERT_TRUE(surfaces.empty());
  ASSERT_TRUE(points.empty());

  const Sphere sphere(0.5);
  // Distance between sphere centers to guarantee collision.
  const double d = sphere.radius() * 2 * 0.9;

  ProximityProperties props;
  AddCompliantHydroelasticProperties(0.5, 1e-8, &props);
  const GeometryId id_A1 = AddAnchored(sphere, V3{d, 0, 0}, props);
  const GeometryId id_D2 = AddDynamic(sphere, V3{2 * d, 0, 0}, props);
  const GeometryId id_D3 = AddDynamic(sphere, V3{3 * d, 0, 0}, props);
  // We don't save id_A4; it's never referenced.
  AddAnchored(sphere, V3{d, -d, 0}, props);

  // (2)-(4).
  // We get the (3) A-D and (2) D-D result but not an (4) A-A result.
  eval_dut(kTriangle);
  const vector<ContactSurface<double>> results_tri = surfaces;
  ASSERT_EQ(surface_ids(results_tri),
            IdPairs({{id_A1, id_D2}, {id_D2, id_D3}}));
  ASSERT_TRUE(points.empty());

  // (5) - consistent ordering in results.
  eval_dut(kPolygon);
  const vector<ContactSurface<double>> results_poly = surfaces;
  ASSERT_EQ(surface_ids(results_poly), surface_ids(results_tri));
  ASSERT_TRUE(points.empty());

  // (6) - representation respected.
  for (const auto& result : results_poly) {
    EXPECT_FALSE(result.is_triangle());
  }
  for (const auto& result : results_tri) {
    EXPECT_TRUE(result.is_triangle());
  }

  // (7) - hydro-incompatible geometries produces point pair.
  const GeometryId id_D5 =
      AddDynamic(sphere, V3{4 * d, 0, 0});  // No hydro props.
  eval_dut(kTriangle);
  ASSERT_EQ(surface_ids(surfaces), IdPairs({{id_A1, id_D2}, {id_D2, id_D3}}));
  ASSERT_EQ(point_ids(points), IdPairs({{id_D3, id_D5}}));

  // (8) - filters are respected.
  const FilterId filter_id = engine_.collision_filter().ApplyTransient(
      CollisionFilterDeclaration().ExcludeWithin(GeometrySet()),
      [id_D2, id_D3, id_D5](const GeometrySet&, CollisionFilterScope) {
        return std::unordered_set<GeometryId>{{id_D2, id_D3, id_D5}};
      });
  eval_dut(kTriangle);
  // The filter removed one contact surface and one point pair.
  ASSERT_EQ(surfaces.size(), 1);
  ASSERT_EQ(points.size(), 0);
  engine_.collision_filter().RemoveDeclaration(filter_id);

  // (9) - AutoDiff derivatives pass through successfully.
  const auto ad_engine = engine_.ToScalarType<AutoDiffXd>();
  unordered_map<GeometryId, RigidTransform<AutoDiffXd>> X_WGs_ad;
  for (const auto& [id, X_WG] : X_WGs_) {
    X_WGs_ad[id] = X_WG.cast<AutoDiffXd>();
  }
  // Seed id_D3's translation with derivatives - the second contact surface and
  // only point pair both involve D3.
  X_WGs_ad[id_D3] = RigidTransform<AutoDiffXd>{
      math::InitializeAutoDiff(X_WGs_.at(id_D3).translation())};

  ad_engine->UpdateWorldPoses(X_WGs_ad);
  vector<ContactSurface<AutoDiffXd>> surfaces_ad;
  vector<PenetrationAsPointPair<AutoDiffXd>> points_ad;
  ad_engine->ComputeContactSurfacesWithFallback(kTriangle, X_WGs_ad,
                                                &surfaces_ad, &points_ad);
  // Still get two results.
  ASSERT_EQ(surfaces_ad.size(), 2);
  ASSERT_EQ(points_ad.size(), 1);
  // Confirm derivatives on the surface and point result.
  const auto& surface = surfaces_ad[1];
  const auto& surf_deriv = surface.total_area().derivatives();
  ASSERT_EQ(surf_deriv.size(), 3);
  EXPECT_FALSE(surf_deriv.isZero());

  const auto& point = points_ad[0];
  const auto& point_deriv = point.depth.derivatives();
  ASSERT_EQ(point_deriv.size(), 3);
  EXPECT_FALSE(point_deriv.isZero());
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
