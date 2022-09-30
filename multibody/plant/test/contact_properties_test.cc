#include "drake/multibody/plant/contact_properties.h"

#include <gtest/gtest.h>

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
constexpr double kTauD = -0.2;
constexpr double kTauDefault = 0.3;

class ContactPropertiesTest : public ::testing::Test {
 public:
  void SetUp() override {
    const SourceId s_id = scene_graph_.RegisterSource("source");
    const FrameId f_id =
        scene_graph_.RegisterFrame(s_id, GeometryFrame("frame"));
    unique_ptr<GeometryInstance> geometry_A =
        MakeGeometryInstance("A", kMuA, kKA, kTauA);
    unique_ptr<GeometryInstance> geometry_B =
        MakeGeometryInstance("B", kMuB, std::nullopt, kTauB);
    unique_ptr<GeometryInstance> geometry_C =
        MakeGeometryInstance("C", kMuC, kKC, std::nullopt);
    unique_ptr<GeometryInstance> geometry_D =
        MakeGeometryInstance("D", kMuD, std::nullopt, kTauD);

    g_A_ = scene_graph_.RegisterGeometry(s_id, f_id, move(geometry_A));
    g_B_ = scene_graph_.RegisterGeometry(s_id, f_id, move(geometry_B));
    g_C_ = scene_graph_.RegisterGeometry(s_id, f_id, move(geometry_C));
    g_D_ = scene_graph_.RegisterGeometry(s_id, f_id, move(geometry_D));
  }

  GeometryId g_A_;  // Has mu, k, tau.
  GeometryId g_B_;  // Has mu, tau.
  GeometryId g_C_;  // Has mu, k.
  GeometryId g_D_;  // Has mu, invalid tau.

  const SceneGraphInspector<double>& inspector() const {
    return scene_graph_.model_inspector();
  }

  unique_ptr<GeometryInstance> MakeGeometryInstance(
      const std::string& name, const CoulombFriction<double>& mu,
      const std::optional<double>& k, const std::optional<double>& tau) {
    auto sphere = make_unique<Sphere>(1.0);
    auto result =
        make_unique<GeometryInstance>(RigidTransformd(), move(sphere), name);
    geometry::ProximityProperties props;
    geometry::AddContactMaterial({}, k, mu, &props);
    if (tau.has_value()) {
      props.AddProperty(geometry::internal::kMaterialGroup,
                        geometry::internal::kRelaxationTime, *tau);
    }
    result->set_proximity_properties(move(props));
    return result;
  }

 private:
  SceneGraph<double> scene_graph_{};
};

TEST_F(ContactPropertiesTest, GetPointContactStiffness) {
  EXPECT_EQ(GetPointContactStiffness(g_A_, kKDefault, inspector()), kKA);
  EXPECT_EQ(GetPointContactStiffness(g_B_, kKDefault, inspector()), kKDefault);
  EXPECT_EQ(GetPointContactStiffness(g_C_, kKDefault, inspector()), kKC);
}

TEST_F(ContactPropertiesTest, GetDissipationTimeConstant) {
  EXPECT_EQ(GetDissipationTimeConstant(g_A_, kTauDefault, inspector(), "A"),
            kTauA);
  EXPECT_EQ(GetDissipationTimeConstant(g_C_, kTauDefault, inspector(), "C"),
            kTauDefault);
  DRAKE_EXPECT_THROWS_MESSAGE(
      GetDissipationTimeConstant(g_D_, kTauDefault, inspector(), "D"),
      ".*relaxation_time = -0.2.*D.*");
}

TEST_F(ContactPropertiesTest, GetCoulombFriction) {
  EXPECT_EQ(GetCoulombFriction(g_A_, inspector()), kMuA);
}

TEST_F(ContactPropertiesTest, GetCombinedPointContactStiffness) {
  EXPECT_EQ(
      GetCombinedPointContactStiffness(g_A_, g_C_, kKDefault, inspector()),
      kKA * kKC / (kKA + kKC));
  EXPECT_EQ(
      GetCombinedPointContactStiffness(g_A_, g_B_, kKDefault, inspector()),
      kKA * kKDefault / (kKA + kKDefault));
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
}

TEST_F(ContactPropertiesTest, GetCombinedDynamicCoulombFriction) {
  EXPECT_EQ(
      GetCombinedDynamicCoulombFriction(g_A_, g_D_, inspector()),
      CalcContactFrictionFromSurfaceProperties(kMuA, kMuD).dynamic_friction());
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
