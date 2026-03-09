#include "drake/multibody/hydroelastics/hydroelastic_engine.h"

#include <limits>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace multibody {
namespace hydroelastics {
namespace internal {
namespace {

using geometry::FrameId;
using geometry::GeometryFrame;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::ProximityProperties;
using geometry::SceneGraph;
using geometry::SceneGraphInspector;
using geometry::SourceId;
using geometry::Sphere;
using math::RigidTransformd;
using std::make_unique;

class HydroelasticEngineTest : public ::testing::Test {
 protected:
  void SetUp() override { source_id_ = scene_graph_.RegisterSource("test"); }

  /** Adds a geometry with the given name to the scene graph (its frame name
   is named "name_frame".)  */
  GeometryId AddGeometry(const std::string& name) {
    FrameId frame_id =
        scene_graph_.RegisterFrame(source_id_, GeometryFrame(name + "_frame"));
    GeometryId id = scene_graph_.RegisterGeometry(
        source_id_, frame_id,
        make_unique<GeometryInstance>(RigidTransformd{},
                                      make_unique<Sphere>(1.0), name));
    return id;
  }

  /** Adds a geometry with the given name and assigns compliant hydroelastic
   geometry properties.  */
  GeometryId AddCompliantGeometry(const std::string& name,
                                  double hydroelastic_modulus,
                                  double dissipation) {
    GeometryId id = AddGeometry(name);
    ProximityProperties props;
    geometry::AddCompliantHydroelasticProperties(/* resolution_hint */ 1.0,
                                                 hydroelastic_modulus, &props);
    geometry::AddContactMaterial(dissipation, {}, {}, &props);
    scene_graph_.AssignRole(source_id_, id, props);
    return id;
  }

  /** Adds a geometry with the given name and assigns rigid hydroelastic
   geometry properties.  */
  GeometryId AddRigidGeometry(const std::string& name) {
    // NOTE: In this formulation, shapes without any definition of elastic
    // modulus or dissipation default to being rigid. Only the presence of
    // proximity properties are required.
    GeometryId id = AddGeometry(name);
    ProximityProperties props;
    geometry::AddRigidHydroelasticProperties(/* resolution_hint */ 1.0, &props);
    scene_graph_.AssignRole(source_id_, id, props);
    return id;
  }

  /** Adds a geometry with the given name and assigns rigid hydroelastic
   geometry properties as well as a value for hydroelastic_modulus.  */
  GeometryId AddRigidGeometryWithElasticModulus(const std::string& name,
                                                double hydroelastic_modulus) {
    // NOTE: For geometries defined to be rigid, HydroelasticEngine will ignore
    // any stored hydroelastic_modulus values and assume infinity.
    GeometryId id = AddGeometry(name);
    ProximityProperties props;
    geometry::AddRigidHydroelasticProperties(/* resolution_hint */ 1.0, &props);
    props.AddProperty(geometry::internal::kHydroGroup,
                      geometry::internal::kElastic, hydroelastic_modulus);
    scene_graph_.AssignRole(source_id_, id, props);
    return id;
  }

  /** Returns a model inspector for this class's scene graph.  */
  const SceneGraphInspector<double>& inspector() const {
    return scene_graph_.model_inspector();
  }

  SceneGraph<double> scene_graph_;
  SourceId source_id_;
};

TEST_F(HydroelasticEngineTest, RigidHydroelasticModulusIgnored) {
  const double E_A = 10.0;  // Pa
  const double E_B = 10.0;  // Pa
  const double d_B = 1.0;   // s/m

  GeometryId id_A = AddRigidGeometryWithElasticModulus("RigidBody", E_A);
  GeometryId id_B = AddCompliantGeometry("SoftBody", E_B, d_B);

  // Create an engine.
  HydroelasticEngine<double> engine;

  // Verify that the combined hydroelastic_modulus (would both geometries be
  // compliant) is not the same as B's hydroelastic modulus.
  double combined_E_if_they_are_both_compliant = (E_A * E_B) / (E_A + E_B);
  ASSERT_NE(E_B, combined_E_if_they_are_both_compliant);

  // The engine will ignore the rigid geometry A's hydroelastic modulus and
  // return B's.
  EXPECT_EQ(engine.CalcCombinedElasticModulus(id_A, id_B, inspector()), E_B);
  EXPECT_EQ(engine.CalcCombinedElasticModulus(id_B, id_A, inspector()), E_B);
}

TEST_F(HydroelasticEngineTest, CombineSoftAndRigidMaterialProperties) {
  const double E_A = 10.0;  // Pa
  const double d_A = 1.0;   // s/m
  GeometryId id_A = AddCompliantGeometry("SoftBody", E_A, d_A);
  GeometryId id_B = AddRigidGeometry("RigidBody");

  // Create an engine.
  HydroelasticEngine<double> engine;

  // The combination must be symmetric.
  EXPECT_EQ(engine.CalcCombinedElasticModulus(id_A, id_B, inspector()), E_A);
  EXPECT_EQ(engine.CalcCombinedElasticModulus(id_B, id_A, inspector()), E_A);
  EXPECT_EQ(engine.CalcCombinedDissipation(id_A, id_B, inspector()), d_A);
  EXPECT_EQ(engine.CalcCombinedDissipation(id_B, id_A, inspector()), d_A);
}

TEST_F(HydroelasticEngineTest, CombineRigidAndRigidMaterialProperties) {
  GeometryId id_A = AddRigidGeometry("RigidBody_A");
  GeometryId id_B = AddRigidGeometry("RigidBody_B");

  // Create an engine.
  HydroelasticEngine<double> engine;

  // The combination must be symmetric.
  const double kInf = std::numeric_limits<double>::infinity();
  EXPECT_EQ(engine.CalcCombinedElasticModulus(id_A, id_B, inspector()), kInf);
  EXPECT_EQ(engine.CalcCombinedElasticModulus(id_B, id_A, inspector()), kInf);
  EXPECT_EQ(engine.CalcCombinedDissipation(id_A, id_B, inspector()), 0.0);
  EXPECT_EQ(engine.CalcCombinedDissipation(id_B, id_A, inspector()), 0.0);
}

TEST_F(HydroelasticEngineTest, CombineSoftAndSoftMaterialProperties) {
  const double E_A = 2.0;  // Pa
  const double d_A = 1.0;  // s/m
  const double E_B = 8.0;  // Pa
  const double d_B = 4.0;  // s/m
  // Expected combined properties.
  const double Estar = 1.6;
  const double dstar = 1.6;
  GeometryId id_A = AddCompliantGeometry("SoftBodyA", E_A, d_A);
  GeometryId id_B = AddCompliantGeometry("SoftBodyB", E_B, d_B);

  // Create an engine.
  HydroelasticEngine<double> engine;

  const double kEps = std::numeric_limits<double>::epsilon();
  // The combination must be symmetric.
  EXPECT_NEAR(engine.CalcCombinedElasticModulus(id_A, id_B, inspector()), Estar,
              kEps);
  EXPECT_NEAR(engine.CalcCombinedDissipation(id_A, id_B, inspector()), dstar,
              kEps);
  EXPECT_NEAR(engine.CalcCombinedElasticModulus(id_B, id_A, inspector()), Estar,
              kEps);
  EXPECT_NEAR(engine.CalcCombinedDissipation(id_B, id_A, inspector()), dstar,
              kEps);
}

TEST_F(HydroelasticEngineTest, MissingProximityPropertiesThrows) {
  GeometryId id_A = AddGeometry("geometry_A");
  GeometryId id_B = AddGeometry("geometry_B");
  GeometryId id_C = AddRigidGeometry("geometry_C");

  // Create an engine.
  HydroelasticEngine<double> engine;

  // Case: Neither have properties.
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.CalcCombinedElasticModulus(id_A, id_B, inspector()),
      "Unable to get the material properties .+ it has no proximity properties "
      "assigned.*");

  // Case: First lacking properties.
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.CalcCombinedElasticModulus(id_A, id_C, inspector()),
      "Unable to get the material properties .+ it has no proximity properties "
      "assigned.*");

  // Case: Second lacking properties.
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine.CalcCombinedElasticModulus(id_C, id_B, inspector()),
      "Unable to get the material properties .+ it has no proximity properties "
      "assigned.*");
}

}  // namespace
}  // namespace internal
}  // namespace hydroelastics
}  // namespace multibody
}  // namespace drake
