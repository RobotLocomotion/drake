#include "drake/multibody/plant/contact_properties.h"

#include <exception>

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
using std::move;
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

    g_A_ = scene_graph_.RegisterGeometry(s_id, f_id, move(geometry_A));
    g_B_ = scene_graph_.RegisterGeometry(s_id, f_id, move(geometry_B));
    g_C_ = scene_graph_.RegisterGeometry(s_id, f_id, move(geometry_C));
    g_D_ = scene_graph_.RegisterGeometry(s_id, f_id, move(geometry_D));

    scene_graph_ad_ =
        dynamic_pointer_cast<SceneGraph<AutoDiffXd>>(
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

  const SceneGraphInspector<double>& inspector() const {
    return scene_graph_.model_inspector();
  }

  unique_ptr<GeometryInstance> MakeGeometryInstance(
      const std::string& name, const CoulombFriction<double>& mu,
      const std::optional<double>& k, const std::optional<double>& tau,
      const std::optional<double>& hunt_crossley_dissipation,
      const std::optional<double>& hydro_modulus = std::nullopt,
      const std::optional<geometry::internal::HydroelasticType>&
          compliance_type = std::nullopt) {
    auto sphere = make_unique<Sphere>(1.0);
    auto result =
        make_unique<GeometryInstance>(RigidTransformd(), move(sphere), name);
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
    result->set_proximity_properties(move(props));
    return result;
  }

  /*
  Compares `extract_from_inspector` for both `T=double` and `T=AutoDiffXd`.
  The function should be of the form:
      extract_from_inspector(T{}, const SceneGraphInspector<T>&)
  The `T{}` argument is for lambda-based argument deduction (pending lambda
  template arguments in C++20).
  */
  template <typename Lambda>
  void CompareAcrossScalarTypes(const Lambda& extract_from_inspector) {
    const double value = extract_from_inspector(double{}, inspector());
    AutoDiffXd value_ad;
    try {
      value_ad = extract_from_inspector(
          AutoDiffXd{}, scene_graph_ad_->model_inspector());
    } catch (...) {
      // N.B. SCOPED_TRACE does not help when an exception occurs.
      drake::log()->error(
          "Exception encountered when trying to extract value from "
          "SceneGraphInspector<AutoDiffXd>");
      std::rethrow_exception(std::current_exception());
    }
    EXPECT_EQ(value_ad, value);
  }

 private:
  SceneGraph<double> scene_graph_{};
  std::unique_ptr<SceneGraph<AutoDiffXd>> scene_graph_ad_{};
};

TEST_F(ContactPropertiesTest, GetPointContactStiffness) {
  EXPECT_EQ(GetPointContactStiffness(g_A_, kKDefault, inspector()), kKA);
  EXPECT_EQ(GetPointContactStiffness(g_B_, kKDefault, inspector()), kKDefault);
  EXPECT_EQ(GetPointContactStiffness(g_C_, kKDefault, inspector()), kKC);
  CompareAcrossScalarTypes(
      [this](auto T_dummy, const auto& inspector) {
        return GetPointContactStiffness(g_A_, kKDefault, inspector);
      });
}

TEST_F(ContactPropertiesTest, GetHydroelasticModulus) {
  EXPECT_EQ(GetHydroelasticModulus(g_A_, kHydroModulusDefault, inspector()),
            kHydroModulusA);
  EXPECT_EQ(GetHydroelasticModulus(g_B_, kHydroModulusDefault, inspector()),
            kInfinity);
  EXPECT_EQ(GetHydroelasticModulus(g_C_, kHydroModulusDefault, inspector()),
            kHydroModulusC);
  EXPECT_EQ(GetHydroelasticModulus(g_D_, kHydroModulusDefault, inspector()),
            kHydroModulusDefault);
  CompareAcrossScalarTypes(
      [this](auto T_dummy, const auto& inspector) {
        return GetHydroelasticModulus(g_A_, kHydroModulusDefault, inspector);
      });
}

TEST_F(ContactPropertiesTest, GetHuntCrossleyDissipation) {
  EXPECT_EQ(GetHuntCrossleyDissipation(g_A_, kHuntCrossleyDissipationDefault,
                                       inspector()),
            kHuntCrossleyDissipationA);
  EXPECT_EQ(GetHuntCrossleyDissipation(g_B_, kHuntCrossleyDissipationDefault,
                                       inspector()),
            kHuntCrossleyDissipationDefault);
  EXPECT_EQ(GetHuntCrossleyDissipation(g_C_, kHuntCrossleyDissipationDefault,
                                       inspector()),
            kHuntCrossleyDissipationDefault);
  CompareAcrossScalarTypes(
      [this](auto T_dummy, const auto& inspector) {
        return GetHuntCrossleyDissipation(
            g_A_, kHuntCrossleyDissipationDefault, inspector);
      });
}

TEST_F(ContactPropertiesTest, GetCombinedHuntCrossleyDissipation) {
  EXPECT_EQ(GetCombinedHuntCrossleyDissipation(
                g_A_, g_B_, kHydroModulusA, kInfinity,
                kHuntCrossleyDissipationDefault, inspector()),
            kHuntCrossleyDissipationA);
  EXPECT_EQ(GetCombinedHuntCrossleyDissipation(
                g_B_, g_A_, kInfinity, kHydroModulusA,
                kHuntCrossleyDissipationDefault, inspector()),
            kHuntCrossleyDissipationA);
  const double dAC = (kHydroModulusC / (kHydroModulusA + kHydroModulusC)) *
                         kHuntCrossleyDissipationA +
                     (kHydroModulusA / (kHydroModulusA + kHydroModulusC)) *
                         kHuntCrossleyDissipationDefault;
  EXPECT_EQ(GetCombinedHuntCrossleyDissipation(
                g_A_, g_C_, kHydroModulusA, kHydroModulusDefault,
                kHuntCrossleyDissipationDefault, inspector()),
            dAC);
  CompareAcrossScalarTypes(
      [this](auto T_dummy, const auto& inspector) {
        using T = decltype(T_dummy);
        return GetCombinedHuntCrossleyDissipation(
            g_A_, g_B_, T(kHydroModulusA), T(kInfinity),
            kHuntCrossleyDissipationDefault, inspector);
      });
}

TEST_F(ContactPropertiesTest, GetDissipationTimeConstant) {
  EXPECT_EQ(GetDissipationTimeConstant(g_A_, kTauDefault, inspector(), "A"),
            kTauA);
  EXPECT_EQ(GetDissipationTimeConstant(g_C_, kTauDefault, inspector(), "C"),
            kTauDefault);
  DRAKE_EXPECT_THROWS_MESSAGE(
      GetDissipationTimeConstant(g_D_, kTauDefault, inspector(), "D"),
      ".*relaxation_time = -0.2.*D.*");
  CompareAcrossScalarTypes(
      [this](auto T_dummy, const auto& inspector) {
        return GetDissipationTimeConstant(g_A_, kTauDefault, inspector, "A");
      });
}

TEST_F(ContactPropertiesTest, GetCoulombFriction) {
  EXPECT_EQ(GetCoulombFriction(g_A_, inspector()), kMuA);
  CompareAcrossScalarTypes(
      [this](auto T_dummy, const auto& inspector) {
        using T = decltype(T_dummy);
        const auto data = GetCoulombFriction(g_A_, inspector);
        // Sum the values as a *very* naive way to check general convresion.
        T sum = data.static_friction() + data.dynamic_friction();
        return sum;
      });
}

TEST_F(ContactPropertiesTest, GetCombinedPointContactStiffness) {
  EXPECT_EQ(
      GetCombinedPointContactStiffness(g_A_, g_C_, kKDefault, inspector()),
      kKA * kKC / (kKA + kKC));
  EXPECT_EQ(
      GetCombinedPointContactStiffness(g_A_, g_B_, kKDefault, inspector()),
      kKA * kKDefault / (kKA + kKDefault));
  CompareAcrossScalarTypes(
      [this](auto T_dummy, const auto& inspector) {
        return GetCombinedPointContactStiffness(
            g_A_, g_C_, kKDefault, inspector);
      });
}

TEST_F(ContactPropertiesTest, GetCombinedDissipationTimeConstant) {
  EXPECT_EQ(GetCombinedDissipationTimeConstant(g_A_, g_B_, kTauDefault, "A",
                                               "B", inspector()),
            kTauA + kTauB);
  EXPECT_EQ(GetCombinedDissipationTimeConstant(g_A_, g_C_, kTauDefault, "A",
                                               "C", inspector()),
            kTauA + kTauDefault);
  DRAKE_EXPECT_THROWS_MESSAGE(
      GetCombinedDissipationTimeConstant(g_A_, g_D_, kTauDefault, "A", "D",
                                         inspector()),
      ".*relaxation_time = -0.2.*D.*");
  CompareAcrossScalarTypes(
      [this](auto T_dummy, const auto& inspector) {
        return GetCombinedDissipationTimeConstant(
            g_A_, g_B_, kTauDefault, "A", "B", inspector);
      });
}

TEST_F(ContactPropertiesTest, GetCombinedDynamicCoulombFriction) {
  EXPECT_EQ(
      GetCombinedDynamicCoulombFriction(g_A_, g_D_, inspector()),
      CalcContactFrictionFromSurfaceProperties(kMuA, kMuD).dynamic_friction());
  CompareAcrossScalarTypes(
      [this](auto T_dummy, const auto& inspector) {
        return GetCombinedDynamicCoulombFriction(g_A_, g_D_, inspector);
      });
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
