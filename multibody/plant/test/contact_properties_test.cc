#include "drake/multibody/plant/contact_properties.h"

#include <limits>
#include <memory>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/pointer_cast.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/scene_graph.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using geometry::FrameId;
using geometry::GeometryFrame;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::SceneGraph;
using geometry::SceneGraphInspector;
using geometry::SourceId;
using geometry::Sphere;
using math::RigidTransformd;
using std::make_unique;
using std::unique_ptr;

const CoulombFriction<double> kMuA(1.0, 1.0);
const CoulombFriction<double> kMuB(2.0, 2.0);
const CoulombFriction<double> kMuC(3.0, 3.0);
const CoulombFriction<double> kMuD(4.0, 4.0);
constexpr double kKA = 1e4;
constexpr double kKC = 2e4;
constexpr double kKDefault = 3e4;
constexpr double kTauA = 0.1;
constexpr double kTauB = 0.2;
constexpr double kNegativeTauD = -0.2;
constexpr double kTauDefault = 0.3;
constexpr double kHydroModulusA = 1e6;
constexpr double kHydroModulusB = 1e8;
constexpr double kHydroModulusC = 1e10;
constexpr double kHydroModulusDefault = 3e6;
constexpr double kHuntCrossleyDissipationA = 0.1;
constexpr double kHuntCrossleyDissipationDefault = 0.1;
constexpr double kInfinity = std::numeric_limits<double>::infinity();

class ContactPropertiesTest : public ::testing::Test {
 public:
  void SetUp() override {
    const SourceId s_id = scene_graph_.RegisterSource("source");
    const FrameId f_id =
        scene_graph_.RegisterFrame(s_id, GeometryFrame("frame"));
    unique_ptr<GeometryInstance> geometry_A = MakeGeometryInstance(
        "A", kMuA, kKA, kTauA, kHuntCrossleyDissipationA, kHydroModulusA);
    unique_ptr<GeometryInstance> geometry_B = MakeGeometryInstance(
        "B", kMuB, std::nullopt, kTauB, std::nullopt, kHydroModulusB,
        geometry::internal::HydroelasticType::kRigid);
    unique_ptr<GeometryInstance> geometry_C = MakeGeometryInstance(
        "C", kMuC, kKC, std::nullopt, std::nullopt, kHydroModulusC,
        geometry::internal::HydroelasticType::kSoft);
    unique_ptr<GeometryInstance> geometry_D = MakeGeometryInstance(
        "D", kMuD, std::nullopt, kNegativeTauD, std::nullopt);

    g_A_ = scene_graph_.RegisterGeometry(s_id, f_id, std::move(geometry_A));
    g_B_ = scene_graph_.RegisterGeometry(s_id, f_id, std::move(geometry_B));
    g_C_ = scene_graph_.RegisterGeometry(s_id, f_id, std::move(geometry_C));
    g_D_ = scene_graph_.RegisterGeometry(s_id, f_id, std::move(geometry_D));

    scene_graph_ad_ = dynamic_pointer_cast<SceneGraph<AutoDiffXd>>(
        scene_graph_.ToAutoDiffXd());
  }

  // mu  = Coulomb friction
  // k   = stiffness
  // tau = relaxation time
  // Eh  = hydroelastic modulus
  // d   = dissipation
  GeometryId g_A_;  // Has mu, k, tau, dA, Eh_A.
  GeometryId g_B_;  // Has mu,    tau,     Eh_B and "rigid hydroelastic".
  GeometryId g_C_;  // Has mu, k,          Eh_C and "compliant hydroelastic".
  GeometryId g_D_;  // Has mu,    invalid tau.

  const SceneGraphInspector<double>& inspector_d() const {
    return scene_graph_.model_inspector();
  }

  const SceneGraphInspector<AutoDiffXd>& inspector_ad() const {
    return scene_graph_ad_->model_inspector();
  }

  unique_ptr<GeometryInstance> MakeGeometryInstance(
      const std::string& name, const CoulombFriction<double>& mu,
      const std::optional<double>& k, const std::optional<double>& tau,
      const std::optional<double>& hunt_crossley_dissipation,
      const std::optional<double>& hydro_modulus = std::nullopt,
      const std::optional<geometry::internal::HydroelasticType>&
          compliance_type = std::nullopt) {
    auto result =
        make_unique<GeometryInstance>(RigidTransformd(), Sphere(1.0), name);
    geometry::ProximityProperties props;
    geometry::AddContactMaterial(hunt_crossley_dissipation, k, mu, &props);
    if (tau.has_value()) {
      props.AddProperty(geometry::internal::kMaterialGroup,
                        geometry::internal::kRelaxationTime, *tau);
    }

    // Hydroelastic contact requires the resolution hint, even if not used for
    // these tests.
    constexpr double kRezHint = 1.0;
    props.AddProperty(geometry::internal::kHydroGroup,
                      geometry::internal::kRezHint, kRezHint);
    if (hydro_modulus.has_value()) {
      props.AddProperty(geometry::internal::kHydroGroup,
                        geometry::internal::kElastic, *hydro_modulus);
    }
    if (compliance_type.has_value()) {
      props.AddProperty(geometry::internal::kHydroGroup,
                        geometry::internal::kComplianceType, *compliance_type);
    }
    result->set_proximity_properties(std::move(props));
    return result;
  }

 private:
  SceneGraph<double> scene_graph_{};
  std::unique_ptr<SceneGraph<AutoDiffXd>> scene_graph_ad_{};
};

/* TODO(SeanCurtis-TRI): These tests don't actually *confirm* scalar conversion.
 They merely confirm that I can invoke the method on inspectors of two scalar
 types and successfully compare the result. That would pass if the return value
 were always doubles. We need to beef these up to confirm the *type*. */

/* These APIs implicitly convert stored double-value to T-values upon request.
 These tests verify that behavior - the last test clause in each function. When
 the GeometryProperties get scalar converted automatically, that extra test
 can go away. Each API must include this scalar test. */

TEST_F(ContactPropertiesTest, GetPointContactStiffness) {
  EXPECT_EQ(GetPointContactStiffness(g_A_, kKDefault, inspector_d()), kKA);
  EXPECT_EQ(GetPointContactStiffness(g_B_, kKDefault, inspector_d()),
            kKDefault);
  EXPECT_EQ(GetPointContactStiffness(g_C_, kKDefault, inspector_d()), kKC);

  EXPECT_EQ(GetPointContactStiffness(g_A_, kKDefault, inspector_d()),
            GetPointContactStiffness(g_A_, kKDefault, inspector_ad()));
}

TEST_F(ContactPropertiesTest, GetHydroelasticModulus) {
  EXPECT_EQ(GetHydroelasticModulus(g_A_, kHydroModulusDefault, inspector_d()),
            kHydroModulusA);
  EXPECT_EQ(GetHydroelasticModulus(g_B_, kHydroModulusDefault, inspector_d()),
            kInfinity);
  EXPECT_EQ(GetHydroelasticModulus(g_C_, kHydroModulusDefault, inspector_d()),
            kHydroModulusC);
  EXPECT_EQ(GetHydroelasticModulus(g_D_, kHydroModulusDefault, inspector_d()),
            kHydroModulusDefault);

  EXPECT_EQ(GetHydroelasticModulus(g_A_, kHydroModulusDefault, inspector_d()),
            GetHydroelasticModulus(g_A_, kHydroModulusDefault, inspector_ad()));
}

TEST_F(ContactPropertiesTest, GetHuntCrossleyDissipation) {
  EXPECT_EQ(GetHuntCrossleyDissipation(g_A_, kHuntCrossleyDissipationDefault,
                                       inspector_d()),
            kHuntCrossleyDissipationA);
  EXPECT_EQ(GetHuntCrossleyDissipation(g_B_, kHuntCrossleyDissipationDefault,
                                       inspector_d()),
            kHuntCrossleyDissipationDefault);
  EXPECT_EQ(GetHuntCrossleyDissipation(g_C_, kHuntCrossleyDissipationDefault,
                                       inspector_d()),
            kHuntCrossleyDissipationDefault);

  EXPECT_EQ(GetHuntCrossleyDissipation(g_A_, kHuntCrossleyDissipationDefault,
                                       inspector_d()),
            GetHuntCrossleyDissipation(g_A_, kHuntCrossleyDissipationDefault,
                                       inspector_ad()));
}

TEST_F(ContactPropertiesTest, GetCombinedHuntCrossleyDissipation) {
  EXPECT_EQ(GetCombinedHuntCrossleyDissipation(
                g_A_, g_B_, kHydroModulusA, kInfinity,
                kHuntCrossleyDissipationDefault, inspector_d()),
            kHuntCrossleyDissipationA);
  EXPECT_EQ(GetCombinedHuntCrossleyDissipation(
                g_B_, g_A_, kInfinity, kHydroModulusA,
                kHuntCrossleyDissipationDefault, inspector_d()),
            kHuntCrossleyDissipationA);
  const double dAC = (kHydroModulusC / (kHydroModulusA + kHydroModulusC)) *
                         kHuntCrossleyDissipationA +
                     (kHydroModulusA / (kHydroModulusA + kHydroModulusC)) *
                         kHuntCrossleyDissipationDefault;
  EXPECT_EQ(GetCombinedHuntCrossleyDissipation(
                g_A_, g_C_, kHydroModulusA, kHydroModulusDefault,
                kHuntCrossleyDissipationDefault, inspector_d()),
            dAC);

  EXPECT_EQ(GetCombinedHuntCrossleyDissipation(
                g_A_, g_B_, kHydroModulusA, kInfinity,
                kHuntCrossleyDissipationDefault, inspector_d()),
            GetCombinedHuntCrossleyDissipation<AutoDiffXd>(
                g_A_, g_B_, kHydroModulusA, kInfinity,
                kHuntCrossleyDissipationDefault, inspector_ad()));
}

TEST_F(ContactPropertiesTest, GetDissipationTimeConstant) {
  EXPECT_EQ(GetDissipationTimeConstant(g_A_, kTauDefault, inspector_d(), "A"),
            kTauA);
  EXPECT_EQ(GetDissipationTimeConstant(g_C_, kTauDefault, inspector_d(), "C"),
            kTauDefault);
  DRAKE_EXPECT_THROWS_MESSAGE(
      GetDissipationTimeConstant(g_D_, kTauDefault, inspector_d(), "D"),
      ".*relaxation_time = -0.2.*D.*");

  EXPECT_EQ(GetDissipationTimeConstant(g_A_, kTauDefault, inspector_d(), "A"),
            GetDissipationTimeConstant(g_A_, kTauDefault, inspector_ad(), "A"));
}

TEST_F(ContactPropertiesTest, GetCoulombFriction) {
  EXPECT_EQ(GetCoulombFriction(g_A_, inspector_d()), kMuA);

  EXPECT_EQ(GetCoulombFriction(g_A_, inspector_d()),
            GetCoulombFriction(g_A_, inspector_ad()));
}

TEST_F(ContactPropertiesTest, GetCombinedPointContactStiffness) {
  // Test the signature that fetches the stiffness values from the geometries.
  EXPECT_EQ(
      GetCombinedPointContactStiffness(g_A_, g_C_, kKDefault, inspector_d()),
      kKA * kKC / (kKA + kKC));
  EXPECT_EQ(
      GetCombinedPointContactStiffness(g_A_, g_B_, kKDefault, inspector_d()),
      kKA * kKDefault / (kKA + kKDefault));

  // Test the signature that performs the underlying computation.
  EXPECT_EQ(GetCombinedPointContactStiffness(kKA, kKC),
            kKA * kKC / (kKA + kKC));

  // Test the case where one geometry is infinitely stiff.
  EXPECT_EQ(GetCombinedPointContactStiffness(
                kKDefault, std::numeric_limits<double>::infinity()),
            kKDefault);
  EXPECT_EQ(GetCombinedPointContactStiffness(
                std::numeric_limits<double>::infinity(), kKDefault),
            kKDefault);

  EXPECT_EQ(
      GetCombinedPointContactStiffness(g_A_, g_C_, kKDefault, inspector_d()),
      GetCombinedPointContactStiffness(g_A_, g_C_, kKDefault, inspector_ad()));
}

TEST_F(ContactPropertiesTest, GetCombinedDissipationTimeConstant) {
  EXPECT_EQ(GetCombinedDissipationTimeConstant(g_A_, g_B_, kTauDefault, "A",
                                               "B", inspector_d()),
            kTauA + kTauB);
  EXPECT_EQ(GetCombinedDissipationTimeConstant(g_A_, g_C_, kTauDefault, "A",
                                               "C", inspector_d()),
            kTauA + kTauDefault);
  DRAKE_EXPECT_THROWS_MESSAGE(
      GetCombinedDissipationTimeConstant(g_A_, g_D_, kTauDefault, "A", "D",
                                         inspector_d()),
      ".*relaxation_time = -0.2.*D.*");

  EXPECT_EQ(GetCombinedDissipationTimeConstant(g_A_, g_B_, kTauDefault, "A",
                                               "B", inspector_d()),
            GetCombinedDissipationTimeConstant(g_A_, g_B_, kTauDefault, "A",
                                               "B", inspector_ad()));
}

TEST_F(ContactPropertiesTest, GetCombinedDynamicCoulombFriction) {
  EXPECT_EQ(
      GetCombinedDynamicCoulombFriction(g_A_, g_D_, inspector_d()),
      CalcContactFrictionFromSurfaceProperties(kMuA, kMuD).dynamic_friction());

  EXPECT_EQ(GetCombinedDynamicCoulombFriction(g_A_, g_D_, inspector_d()),
            GetCombinedDynamicCoulombFriction(g_A_, g_D_, inspector_ad()));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
