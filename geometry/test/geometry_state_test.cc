#include "drake/geometry/geometry_state.h"

#include <algorithm>
#include <memory>
#include <set>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/geometry_set.h"
#include "drake/geometry/internal_frame.h"
#include "drake/geometry/render/render_label.h"
#include "drake/geometry/shape_specification.h"
#include "drake/geometry/test_utilities/dummy_render_engine.h"

namespace drake {
namespace geometry {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;
using internal::DummyRenderEngine;
using internal::InternalFrame;
using internal::InternalGeometry;
using render::RenderLabel;
using std::make_unique;
using std::map;
using std::move;
using std::pair;
using std::set;
using std::string;
using std::unique_ptr;
using std::unordered_map;
using std::unordered_set;
using std::vector;

// Implementation of friend class that allows me to peek into the geometry state
// to confirm invariants on the state's internal workings as a result of
// operations.

template <class T>
class GeometryStateTester {
 public:
  void set_state(GeometryState<T>* state) { state_ = state; }

  FrameId get_world_frame() const { return InternalFrame::world_frame_id(); }

  SourceId get_self_source_id() const {
    return state_->self_source_;
  }

  const unordered_map<SourceId, string>& get_source_name_map() const {
    return state_->source_names_;
  }

  const unordered_map<SourceId, FrameIdSet>& get_source_frame_id_map() const {
    return state_->source_frame_id_map_;
  }

  const unordered_map<SourceId, FrameIdSet>& get_source_root_frame_map() const {
    return state_->source_root_frame_map_;
  }

  const unordered_map<SourceId, unordered_set<GeometryId>>&
  get_source_anchored_geometry_map() const {
    return state_->source_anchored_geometry_map_;
  }

  const unordered_map<FrameId, InternalFrame>& get_frames() const {
    return state_->frames_;
  }

  const unordered_map<GeometryId, InternalGeometry>& get_geometries() const {
    return state_->geometries_;
  }

  const vector<GeometryId>& get_geometry_index_id_map() const {
    return state_->geometry_index_to_id_map_;
  }

  const vector<FrameId>& get_frame_index_id_map() const {
    return state_->frame_index_to_id_map_;
  }

  const vector<GeometryIndex>& get_dynamic_pose_index_id_map() const {
    return state_->dynamic_proximity_index_to_internal_map_;
  }

  const vector<Isometry3<T>>& get_geometry_world_poses() const {
    return state_->X_WG_;
  }

  const vector<Isometry3<T>>& get_frame_world_poses() const {
    return state_->X_WF_;
  }

  const vector<Isometry3<T>>& get_frame_parent_poses() const {
    return state_->X_PF_;
  }

  void SetFramePoses(SourceId source_id, const FramePoseVector<T>& poses) {
    state_->SetFramePoses(source_id, poses);
  }

  void FinalizePoseUpdate() {
    state_->FinalizePoseUpdate();
  }

  template <typename ValueType>
  void ValidateFrameIds(SourceId source_id,
                        const FrameKinematicsVector<ValueType>& data) const {
    state_->ValidateFrameIds(source_id, data);
  }

  int peek_next_clique() const {
    return state_->peek_next_clique();
  }

  const InternalGeometry* GetGeometry(GeometryId id) const {
    return state_->GetGeometry(id);
  }

  const render::RenderEngine& GetRenderEngineOrThrow(
      const std::string& name) const {
    return state_->GetRenderEngineOrThrow(name);
  }

  const unordered_map<string, copyable_unique_ptr<render::RenderEngine>>&
  render_engines() const {
    return state_->render_engines_;
  }

 private:
  GeometryState<T>* state_;
};

namespace {

// Class to aid in testing Shape introspection. Instantiated with a model
// Shape instance, it registers a copy of that shape and confirms that the
// introspected Shape matches in type and parameters.
template <typename ShapeType>
class ShapeMatcher final : public ShapeReifier {
 public:
  explicit ShapeMatcher(const ShapeType& expected)
      : expected_(expected), result_(::testing::AssertionFailure()) {}

  // Tests shape introspection.
  ::testing::AssertionResult ShapeIntrospects(GeometryState<double>* state,
                                              SourceId source_id,
                                              FrameId frame_id) {
    GeometryId g_id = state->RegisterGeometry(
        source_id, frame_id,
        make_unique<GeometryInstance>(Isometry3d::Identity(),
                                      make_unique<ShapeType>(expected_),
                                      "shape"));
    state->GetShape(g_id).Reify(this);
    return result_;
  }

  // Shape reifier implementations.
  void ImplementGeometry(const Sphere& sphere, void*) final {
    if (IsExpectedType(sphere)) {
      TestShapeParameters(sphere);
    }
  }

  void ImplementGeometry(const Cylinder& cylinder, void*) final {
    if (IsExpectedType(cylinder)) {
      TestShapeParameters(cylinder);
    }
  }

  void ImplementGeometry(const HalfSpace& half_space, void*) final {
    // Halfspace has no parameters; so no further testing is necessary.
    IsExpectedType(half_space);
  }

  void ImplementGeometry(const Box& box, void*) final {
    if (IsExpectedType(box)) {
      TestShapeParameters(box);
    }
  }

  void ImplementGeometry(const Mesh& mesh, void*) final {
    if (IsExpectedType(mesh)) {
      TestShapeParameters(mesh);
    }
  }

  void ImplementGeometry(const Convex& convex, void*) final {
    if (IsExpectedType(convex)) {
      TestShapeParameters(convex);
    }
  }

 private:
  // Base template signature for comparing shape parameters. By default, it
  // fails.
  template <typename TestType>
  void TestShapeParameters(const TestType& test) {
    error() << "Not implemented for " << NiceTypeName::Get<ShapeType>() << " vs"
            << NiceTypeName::Get<TestType>();
  }

  // Convenience method for logging errors.
  ::testing::AssertionResult error() {
    if (result_) result_ = ::testing::AssertionFailure();
    return result_;
  }

  // Tests type of parameter against reference type. If they match, resets the
  // result to the best known answer (success). Subsequent parameter testing
  // will revert it to false via invocations to error().
  template <typename TestShape>
  bool IsExpectedType(const TestShape& test) {
    if (typeid(ShapeType) == typeid(const TestShape)) {
      result_ = ::testing::AssertionSuccess();
      return true;
    } else {
      result_ << "Expected '" << NiceTypeName::Get<ShapeType>() << "', given '"
              << NiceTypeName::Get<TestShape>() << "'";
      return false;
    }
  }

  // The model shape.
  const ShapeType expected_;

  // The result of the test (with appropriate failure messages).
  ::testing::AssertionResult result_;
};

// Specializations for where the ShapeMatcher's ShapeType match the reified
// TestType.
template <>
template <>
void ShapeMatcher<Sphere>::TestShapeParameters(const Sphere& test) {
  if (test.get_radius() != expected_.get_radius()) {
    error() << "\nExpected sphere radius " << expected_.get_radius() << ", "
            << "received sphere radius " << test.get_radius();
  }
}

template <>
template <>
void ShapeMatcher<Cylinder>::TestShapeParameters(const Cylinder& test) {
  if (test.get_radius() != expected_.get_radius()) {
    error() << "\nExpected cylinder radius " << expected_.get_radius() << ", "
            << "received cylinder radius " << test.get_radius();
  }
  if (test.get_length() != expected_.get_length()) {
    error() << "\nExpected cylinder length " << expected_.get_length()
            << ", received cylinder length " << test.get_length();
  }
}

template <>
template <>
void ShapeMatcher<Box>::TestShapeParameters(const Box& test) {
  if (test.width() != expected_.width()) {
    error() << "\nExpected box width " << expected_.width() << ", "
            << "received box width " << test.width();
  }
  if (test.height() != expected_.height()) {
    error() << "\nExpected box height " << expected_.height()
            << ", received box height " << test.height();
  }
  if (test.depth() != expected_.depth()) {
    error() << "\nExpected box depth " << expected_.depth()
            << ", received box depth " << test.depth();
  }
}

template <>
template <>
void ShapeMatcher<Mesh>::TestShapeParameters(const Mesh& test) {
  if (test.filename() != expected_.filename()) {
    error() << "\nExpected mesh filename " << expected_.filename() << ", "
            << "received mesh filename " << test.filename();
  }
  if (test.scale() != expected_.scale()) {
    error() << "\nExpected mesh scale " << expected_.scale()
            << ", received mesh scale " << test.scale();
  }
}

template <>
template <>
void ShapeMatcher<Convex>::TestShapeParameters(const Convex& test) {
  if (test.filename() != expected_.filename()) {
    error() << "\nExpected convex filename " << expected_.filename() << ", "
            << "received convex filename " << test.filename();
  }
  if (test.scale() != expected_.scale()) {
    error() << "\nExpected convex scale " << expected_.scale()
            << ", received convex scale " << test.scale();
  }
}

// A mask-style enumeration for indicating which of a set of roles should be
// assigned to the newly configured geometry.
enum class Assign {
  kNone = 0,
  kProximity = 1,
  kIllustration = 2,
  kPerception = 4
};

Assign operator&(Assign a, Assign b) {
  return static_cast<Assign>(static_cast<int>(a) & static_cast<int>(b));
}

Assign operator|(Assign a, Assign b) {
  return static_cast<Assign>(static_cast<int>(a) | static_cast<int>(b));
}

// The fundamental base class for the geometry state tests; it provides
// utilities for configuring an owned geometry state. This class allows us to
// create arbitrary test harnesses using the same infrastructure. (See
// GeometryStateTest and RemoveRoleTests -- this latter comes in a future PR.)
class GeometryStateTestBase {
 public:
  GeometryStateTestBase() = default;

  // The initialization to be done prior to each test; the derived test class
  // should invoke this in its SetUp() method.
  void TestInit(bool add_renderer = true) {
    frame_ = make_unique<GeometryFrame>("ref_frame");
    instance_pose_.translation() << 10, 20, 30;
    instance_ = make_unique<GeometryInstance>(
        instance_pose_, make_unique<Sphere>(1.0), "instance");
    gs_tester_.set_state(&geometry_state_);
    if (add_renderer) {
      auto render_engine = make_unique<DummyRenderEngine>();
      render_engine_ = render_engine.get();
      geometry_state_.AddRenderer(kDummyRenderName, move(render_engine));
    }
  }

  // Utility method for adding a source to the state.
  SourceId NewSource(const string& name = "") {
    return geometry_state_.RegisterNewSource(name == "" ? kSourceName : name);
  }

  // This method sets up a dummy tree to facilitate testing. It creates a single
  // source with a fixed configuration of frames with a pre-determined number of
  // geometries per frame.
  //
  //  Creates the following tree:
  //                                      s_id
  //                                        ├───┬────────────┐
  //                                        │   │            │
  //                                       f0   f1           a
  //                                      ╱ │    ├───┬───┐
  //                                    g0  g1   │   │   │
  //                                             f2  g2  g3
  //                                            ╱ ╲
  //                                           g4 g5
  //
  // Default frame configuration
  //  f0 is @ <1, 2, 3>, with a 90-degree rotation around x.
  //  f1 is @ <10, 20, 30>, with a 90-degree rotation around y.
  //  f2 is @ <-10, -20, -30>, with a -90-degree rotation around y.
  //    NOTE: f2's pose is the inverse of f1. As such, for g4 & g5, the pose
  //    relative to the parent frame f2 is the same as to the world, e.g.,
  //    X_PG = X_WG.
  //  a is an anchored box of size (100, 100, 2), positioned at <0, 0 -1> (so
  //     that it's top face lies on the z = 0 plane.
  // Geometry configuration
  //  gi is at position <i + 1, 0, 0>, with a rotation of iπ/2 radians around
  //    the x-axis.
  //  Each of the dynamic geometries are spheres of radius one at the following
  //  positions (expressed in the world frame) (with identity rotation):
  //   p_WG0 = <1, 2, 3>
  //   p_WG1 = <2, 2, 3>
  //   p_WG2 = <10, 20, 33>
  //   p_WG3 = <10, 20, 34>
  //   p_WG4 = <5, 0, 0>
  //   p_WG5 = <6, 0, 0)
  //
  //  In the default configuration, there are only two colliding pairs:
  //    (a, g4) and (a, g5)
  //  Although the sibling geometries affixed to each frame overlap, the pairs
  //  (g0, g1), (g2, g3), and (g4, g5) are implicitly filtered because they are
  //  sibling geometries affixed to the same frame.
  //
  // By default, no roles are assigned to the geometries. However, roles can
  // be assigned to *all* registered geometries by indicating the type of role
  // to assign (via `roles_to_assign`).
  SourceId SetUpSingleSourceTree(Assign roles_to_assign = Assign::kNone) {
    using std::to_string;

    source_id_ = NewSource();

    // Create f0.
    Isometry3d pose = Isometry3d::Identity();
    pose.translation() << 1, 2, 3;
    pose.linear() << 1, 0, 0, 0, 0, 1, 0, -1, 0;  // 90° around x-axis.
    frames_.push_back(geometry_state_.RegisterFrame(
        source_id_, GeometryFrame("f0")));
    X_WFs_.push_back(pose);
    X_PFs_.push_back(pose);

    // Create f1.
    pose.translation() << 10, 20, 30;
    pose.linear() << 0, 0, -1, 0, 1, 0, 1, 0, 0;  // 90° around y-axis.
    frames_.push_back(geometry_state_.RegisterFrame(
        source_id_, GeometryFrame("f1")));
    X_WFs_.push_back(pose);
    X_PFs_.push_back(pose);

    // Create f2.
    pose = pose.inverse();
    frames_.push_back(geometry_state_.RegisterFrame(
        source_id_, frames_[1], GeometryFrame("f2")));
    X_WFs_.push_back(X_WFs_[1] * pose);
    X_PFs_.push_back(pose);

    // Add geometries to each frame.
    const Vector3<double> x_axis(1, 0, 0);
    geometries_.resize(kFrameCount * kGeometryCount);
    geometry_names_.resize(geometries_.size());
    int g_count = 0;
    for (auto frame_id : frames_) {
      for (int i = 0; i < kGeometryCount; ++i) {
        pose.translation() << g_count + 1, 0, 0;
        pose.linear() =
            AngleAxis<double>(g_count * M_PI / 2.0, x_axis).matrix();
        // Have the name reflect the frame and the index in the geometry.
        const string& name =
            to_string(frame_id) + "_g" + to_string(i);
        geometry_names_[g_count] = name;
        geometries_[g_count] = geometry_state_.RegisterGeometry(
            source_id_, frame_id,
            make_unique<GeometryInstance>(pose, make_unique<Sphere>(1), name));
        X_FGs_.push_back(pose);
        ++g_count;
      }
    }

    // Create anchored geometry.
    X_WA_ = Isometry3d{Translation3d{0, 0, -1}};
    // This simultaneously tests the named registration function and
    // _implicitly_ tests registration of geometry against the world frame id
    // (as that is how `RegisterAnchoredGeometry()` works.
    anchored_geometry_ = geometry_state_.RegisterAnchoredGeometry(
        source_id_, make_unique<GeometryInstance>(
            X_WA_, make_unique<Box>(100, 100, 2), anchored_name_));

    if ((roles_to_assign & Assign::kProximity) != Assign::kNone) {
      AssignProximityToSingleSourceTree();
    }

    if ((roles_to_assign & Assign::kPerception) != Assign::kNone) {
      AssignPerceptionToSingleSourceTree();
    }

    if ((roles_to_assign & Assign::kIllustration) != Assign::kNone) {
      AssignIllustrationToSingleSourceTree();
    }

    return source_id_;
  }

  // Reports characteristics of the dummy tree.
  int single_tree_frame_count() const {
    // Added dynamic frames plus the world frame.
    return kFrameCount + 1;
  }

  int single_tree_total_geometry_count() const {
    return single_tree_dynamic_geometry_count() + anchored_geometry_count();
  }

  int single_tree_dynamic_geometry_count() const {
    return kFrameCount * kGeometryCount;
  }

  int anchored_geometry_count() const { return 1; }

  vector<GeometryId> all_geometry_ids() const {
    vector<GeometryId> ids(geometries_);
    ids.push_back(anchored_geometry_);
    return ids;
  }

  int default_collision_pair_count() const {
    // Without filtering, this should be the expected pairs:
    // (a, g4), (a, g5)
    return 2;
  }

  // Members owned by the test class.
  unique_ptr<GeometryFrame> frame_;
  unique_ptr<GeometryInstance> instance_;
  Isometry3d instance_pose_{Isometry3d::Identity()};
  GeometryState<double> geometry_state_;
  GeometryStateTester<double> gs_tester_;
  DummyRenderEngine* render_engine_{};

  // Values for setting up and testing the dummy tree.
  enum Counts {
    kFrameCount = 3,
    kGeometryCount = 2    // Geometries *per frame*.
  };
  // The frame ids created in the dummy tree instantiation.
  vector<FrameId> frames_;
  // TODO(SeanCurtis-TRI): geometries_ and geometry_names_ have long since
  // been invalid names -- with the addition of the anchored geometry, these
  // are now strictly dynamic geometries (and names) and should be renamed
  // accordingly.
  // The geometry ids created in the dummy tree instantiation.
  vector<GeometryId> geometries_;
  // The names for all the geometries (as registered).
  vector<string> geometry_names_;
  // The single, anchored geometry id.
  GeometryId anchored_geometry_;
  // The registered name of the anchored geometry.
  const string anchored_name_{"anchored"};
  // The id of the single-source tree.
  SourceId source_id_;

  // The poses of the frames in the world frame.
  vector<Isometry3d> X_WFs_;
  // The poses of the frames in the parent's frame.
  vector<Isometry3d> X_PFs_;
  // The poses of the dynamic geometries in the parent frame.
  vector<Isometry3d> X_FGs_;
  // The pose of the anchored geometry in the world frame.
  Isometry3d X_WA_;
  // The default source name.
  const string kSourceName{"default_source"};
  // The name of the dummy renderer added to the geometry state.
  const string kDummyRenderName{"dummy_renderer"};

 private:
  // Convenience method for assigning illustration properties to all geometries
  // in the single source tree.
  void AssignProximityToSingleSourceTree() {
    ASSERT_TRUE(source_id_.is_valid());
    ProximityProperties properties;
    AssignRoleToSingleSourceTree(properties);
  }

  // Convenience method for assigning illustration properties to all geometries
  // in the single source tree.
  void AssignIllustrationToSingleSourceTree() {
    ASSERT_TRUE(source_id_.is_valid());
    IllustrationProperties properties;
    properties.AddProperty("phong", "diffuse",
                           Vector4<double>{0.8, 0.8, 0.8, 1.0});
    AssignRoleToSingleSourceTree(properties);
  }

  // Convenience method for assigning perception properties to all geometries
  // in the single source tree.
  void AssignPerceptionToSingleSourceTree() {
    ASSERT_TRUE(source_id_.is_valid());
    // If no render engine has been added, we need to get the accepting
    // properties from a temporary.
    PerceptionProperties properties =
        render_engine_ ? render_engine_->accepting_properties()
                       : DummyRenderEngine().accepting_properties();
    properties.AddProperty("phong", "diffuse",
                           Vector4<double>{0.8, 0.8, 0.8, 1.0});
    properties.AddProperty("label", "id", RenderLabel::kDontCare);
    AssignRoleToSingleSourceTree(properties);
  }

  template <typename PropertyType>
  void AssignRoleToSingleSourceTree(const PropertyType& properties) {
    ASSERT_TRUE(source_id_.is_valid());
    for (GeometryId id : geometries_) {
      geometry_state_.AssignRole(source_id_, id, properties);
    }
    geometry_state_.AssignRole(source_id_, anchored_geometry_, properties);
  }
};

// Class for performing most tests on GeometryState. It does *not* populate the
// GeometryState at all, but relies on each test to do so as appropriate
// (via the SetUpSingleSourceTree() method).
class GeometryStateTest : public GeometryStateTestBase, public ::testing::Test {
 protected:
  void SetUp() override {
    TestInit();
  }
};

// Confirms that a new GeometryState has no data.
TEST_F(GeometryStateTest, Constructor) {
  // GeometryState has a "self source".
  EXPECT_EQ(geometry_state_.get_num_sources(), 1);
  // GeometryState always has a world frame.
  EXPECT_EQ(geometry_state_.get_num_frames(), 1);
  EXPECT_EQ(geometry_state_.get_num_geometries(), 0);
}

// Confirms that the registered shapes are correctly returned upon
// introspection.
TEST_F(GeometryStateTest, IntrospectShapes) {
  const SourceId source_id = geometry_state_.RegisterNewSource("test_source");
  const FrameId frame_id = geometry_state_.RegisterFrame(
      source_id, GeometryFrame("frame"));

  // Test across all valid shapes.
  {
    ShapeMatcher<Sphere> matcher(Sphere(0.25));
    EXPECT_TRUE(
        matcher.ShapeIntrospects(&geometry_state_, source_id, frame_id));
  }
  {
    ShapeMatcher<Cylinder> matcher(Cylinder(0.25, 2.0));
    EXPECT_TRUE(
        matcher.ShapeIntrospects(&geometry_state_, source_id, frame_id));
  }
  {
    ShapeMatcher<Box> matcher(Box(0.25, 2.0, 32.0));
    EXPECT_TRUE(
        matcher.ShapeIntrospects(&geometry_state_, source_id, frame_id));
  }
  {
    ShapeMatcher<HalfSpace> matcher(HalfSpace{});
    EXPECT_TRUE(
        matcher.ShapeIntrospects(&geometry_state_, source_id, frame_id));
  }
  {
    ShapeMatcher<Mesh> matcher(Mesh{"Path/to/mesh", 0.25});
    EXPECT_TRUE(
        matcher.ShapeIntrospects(&geometry_state_, source_id, frame_id));
  }
  {
    ShapeMatcher<Convex> matcher(Convex{"Path/to/convex", 0.25});
    EXPECT_TRUE(
        matcher.ShapeIntrospects(&geometry_state_, source_id, frame_id));
  }

  // Test invalid id.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.GetShape(GeometryId::get_new_id()),
      std::logic_error, "No geometry available for invalid geometry id: .+");
}

// Confirms semantics of user-specified source name.
//    - The source name is stored and retrievable,
//    - duplicate names are detected and considered errors, and
//    - unrecognized source ids do not produce names.
TEST_F(GeometryStateTest, SourceRegistrationWithNames) {
  using std::to_string;

  // Case: Successful registration of unique source id and name.
  SourceId s_id;
  const string name = "Unique";
  EXPECT_NO_THROW((s_id = geometry_state_.RegisterNewSource(name)));
  EXPECT_TRUE(geometry_state_.source_is_registered(s_id));
  EXPECT_EQ(geometry_state_.get_source_name(s_id), name);

  // Case: User-specified name duplicates previously registered name.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterNewSource(name), std::logic_error,
      "Registering new source with duplicate name: Unique.");

  // Case: query with invalid source id.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.get_source_name(SourceId::get_new_id()), std::logic_error,
      "Querying source name for an invalid source id: \\d+.");
}

// Tests the geometry statistics values. It uses the single-source tree to
// create a state with interesting metrics. Also confirms the "is registered"
// -ness of known valid sources and known invalid sources.
TEST_F(GeometryStateTest, GeometryStatistics) {
  const SourceId dummy_source = SetUpSingleSourceTree();

  EXPECT_TRUE(geometry_state_.source_is_registered(dummy_source));
  // Dummy source + self source.
  EXPECT_EQ(geometry_state_.get_num_sources(), 2);
  EXPECT_EQ(geometry_state_.get_num_frames(), single_tree_frame_count());
  EXPECT_EQ(geometry_state_.NumFramesForSource(source_id_),
            single_tree_frame_count() - 1);  // subtract the world frame.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.NumFramesForSource(SourceId::get_new_id()),
      std::logic_error, "Referenced geometry source .* is not registered.");
  EXPECT_EQ(geometry_state_.GetNumDynamicGeometries(),
            single_tree_dynamic_geometry_count());
  EXPECT_EQ(geometry_state_.GetNumAnchoredGeometries(),
            anchored_geometry_count());
  EXPECT_EQ(
      geometry_state_.GetNumAnchoredGeometries(),
      geometry_state_.GetNumFrameGeometries(InternalFrame::world_frame_id()));
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            single_tree_total_geometry_count());
  const SourceId false_id = SourceId::get_new_id();
  EXPECT_FALSE(geometry_state_.source_is_registered(false_id));
}

TEST_F(GeometryStateTest, GetOwningSourceName) {
  SetUpSingleSourceTree();

  EXPECT_EQ(kSourceName, geometry_state_.GetOwningSourceName(frames_[0]));
  EXPECT_EQ(kSourceName, geometry_state_.GetOwningSourceName(geometries_[0]));
  EXPECT_EQ(kSourceName,
            geometry_state_.GetOwningSourceName(anchored_geometry_));

  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.GetOwningSourceName(FrameId::get_new_id()),
      std::logic_error, "Referenced frame .* has not been registered.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.GetOwningSourceName(GeometryId::get_new_id()),
      std::logic_error, "Geometry id .* does not map to a registered geometry");
}

// Compares the autodiff geometry state (embedded in its tester) against the
// double state to confirm they have the same values/topology.
void ExpectSuccessfulTransmogrification(
    const GeometryStateTester<AutoDiffXd>& ad_tester,
    const GeometryStateTester<double>& d_tester) {

  // 1. Test all of the identifier -> trivially testable value maps
  EXPECT_EQ(ad_tester.get_self_source_id(), d_tester.get_self_source_id());
  EXPECT_EQ(ad_tester.get_source_name_map(), d_tester.get_source_name_map());
  EXPECT_EQ(ad_tester.get_source_frame_id_map(),
            d_tester.get_source_frame_id_map());
  EXPECT_EQ(ad_tester.get_source_root_frame_map(),
            d_tester.get_source_root_frame_map());
  EXPECT_EQ(ad_tester.get_source_anchored_geometry_map(),
            d_tester.get_source_anchored_geometry_map());
  EXPECT_EQ(ad_tester.get_geometries(), d_tester.get_geometries());
  EXPECT_EQ(ad_tester.get_frames(), d_tester.get_frames());

  // 2. Test the vectors of ids
  EXPECT_EQ(ad_tester.get_geometry_index_id_map(),
            d_tester.get_geometry_index_id_map());
  EXPECT_EQ(ad_tester.get_frame_index_id_map(),
            d_tester.get_frame_index_id_map());
  EXPECT_EQ(ad_tester.get_dynamic_pose_index_id_map(),
            d_tester.get_dynamic_pose_index_id_map());

  // 3. Compare Isometry3d with Isometry3<AutoDiffXd>
  for (GeometryId id : ad_tester.get_geometry_index_id_map()) {
    EXPECT_TRUE(CompareMatrices(
        ad_tester.get_geometries().at(id).X_FG().matrix().block<3, 4>(0, 0),
        d_tester.get_geometries().at(id).X_FG().matrix().block<3, 4>(0, 0)));
  }

  // 4. Compare Isometry3<AutoDiffXd> with Isometry3d
  auto test_ad_vs_double = [](const vector<Isometry3<AutoDiffXd>>& test,
                              const vector<Isometry3d>& ref) {
    EXPECT_EQ(test.size(), ref.size());
    for (size_t i = 0; i < ref.size(); ++i) {
      for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 4; ++col) {
          EXPECT_EQ(test[i](row, col).value(), ref[i](row, col));
        }
      }
    }
  };

  test_ad_vs_double(ad_tester.get_frame_parent_poses(),
                    d_tester.get_frame_parent_poses());
  test_ad_vs_double(ad_tester.get_geometry_world_poses(),
                    d_tester.get_geometry_world_poses());
  test_ad_vs_double(ad_tester.get_frame_world_poses(),
                    d_tester.get_frame_world_poses());

  // 5. Engine transmogrification tested in its own test; it will *not* be done
  // here.
}

// This tests the ability to assign a GeometryState<double> to a
// GeometryState<T>. Implicitly uses transmogrification.
TEST_F(GeometryStateTest, AssignDoubleToAutoDiff) {
  SetUpSingleSourceTree();
  GeometryState<AutoDiffXd> ad_state{};
  ad_state = geometry_state_;
  GeometryStateTester<AutoDiffXd> ad_tester;
  ad_tester.set_state(&ad_state);
  ExpectSuccessfulTransmogrification(ad_tester, gs_tester_);
}

// Uses the single source tree to confirm that the data has migrated properly.
TEST_F(GeometryStateTest, Transmogrify) {
  SetUpSingleSourceTree();
  unique_ptr<GeometryState<AutoDiffXd>> ad_state =
      geometry_state_.ToAutoDiffXd();
  GeometryStateTester<AutoDiffXd> ad_tester;
  ad_tester.set_state(ad_state.get());
  ExpectSuccessfulTransmogrification(ad_tester, gs_tester_);
}

// Confirms that the actions of initializing the single-source tree leave the
// geometry state in the expected configuration.
TEST_F(GeometryStateTest, ValidateSingleSourceTree) {
  // NOTE: There are *two* sources -- the built-in self source id for geometry
  // state and the id added here.
  const SourceId s_id = SetUpSingleSourceTree();

  // The source has *direct* access to all registered frames.
  {
    const auto& s_f_id_map = gs_tester_.get_source_frame_id_map();
    EXPECT_EQ(s_f_id_map.size(), 2);
    EXPECT_NE(s_f_id_map.find(s_id), s_f_id_map.end());
    const auto &f_id_set = s_f_id_map.at(s_id);
    EXPECT_EQ(frames_.size(), f_id_set.size());
    for (int f = 0; f < kFrameCount; ++f) {
      EXPECT_NE(f_id_set.find(frames_[f]), f_id_set.end());
    }
  }

  // The root map *only* includes the root frames. Frames 0 & 1 *should* be
  // included; frame 2 should *not*.
  {
    const auto& s_root_map = gs_tester_.get_source_root_frame_map();
    EXPECT_EQ(s_root_map.size(), 2);
    EXPECT_NE(s_root_map.find(s_id), s_root_map.end());
    const auto &root_id_set = s_root_map.at(s_id);
    EXPECT_NE(root_id_set.find(frames_[0]), root_id_set.end());
    EXPECT_NE(root_id_set.find(frames_[1]), root_id_set.end());
    EXPECT_EQ(root_id_set.find(frames_[2]), root_id_set.end());
  }

  // The internal frames are what and where they should be.
  {
    using std::to_string;
    const auto& internal_frames = gs_tester_.get_frames();
    // The world frame + the frames added by s_id.
    EXPECT_EQ(internal_frames.size(), kFrameCount + 1);

    auto test_frame = [internal_frames, this, s_id](
        FrameIndex i, FrameId parent_id, int num_child_frames) {
      const auto& frame = internal_frames.at(frames_[i]);
      EXPECT_EQ(frame.source_id(), s_id);
      EXPECT_EQ(frame.id(), frames_[i]);
      EXPECT_EQ(frame.name(), "f" + to_string(i));
      EXPECT_EQ(frame.frame_group(), 0);  // Defaults to zero.
      EXPECT_EQ(frame.index(), i + 1);   // ith frame added.
      EXPECT_EQ(frame.parent_frame_id(), parent_id);
      EXPECT_EQ(frame.child_frames().size(), num_child_frames);
      const auto& child_geometries = frame.child_geometries();
      EXPECT_EQ(child_geometries.size(), 2);
      EXPECT_NE(child_geometries.find(geometries_[i * 2]),
                                      child_geometries.end());
      EXPECT_NE(child_geometries.find(geometries_[i * 2 + 1]),
                                      child_geometries.end());
      const auto& frame_in_parent = gs_tester_.get_frame_parent_poses();
      EXPECT_TRUE(
          CompareMatrices(frame_in_parent[frame.index()].matrix(),
                          X_PFs_[i].matrix()));
    };

    // When added, all frames' poses w.r.t. their parents are the identity.
    const auto& frame_in_parent = gs_tester_.get_frame_parent_poses();
    for (FrameId frame_id : frames_) {
      const auto& frame = internal_frames.at(frame_id);
      EXPECT_TRUE(CompareMatrices(frame_in_parent[frame.index()].matrix(),
                                  Isometry3d::Identity().matrix()));
    }

    // Confirm posing positions the frames properly.
    FramePoseVector<double> poses;
    for (int f = 0; f < static_cast<int>(frames_.size()); ++f) {
      poses.set_value(frames_[f], X_PFs_[f]);
    }
    gs_tester_.SetFramePoses(s_id, poses);
    gs_tester_.FinalizePoseUpdate();

    test_frame(FrameIndex(0), gs_tester_.get_world_frame(), 0);
    test_frame(FrameIndex(1), gs_tester_.get_world_frame(), 1);
    test_frame(FrameIndex(2), frames_[1], 0);
  }

  // The internal geometries are what and where they should be.
  {
    const auto& internal_geometries = gs_tester_.get_geometries();
    EXPECT_EQ(internal_geometries.size(), single_tree_total_geometry_count());
    // NOTE: This relies on the anchored geometry being added *last*.
    for (int i = 0; i < single_tree_dynamic_geometry_count(); ++i) {
      const auto& geometry = internal_geometries.at(geometries_[i]);
      EXPECT_EQ(geometry.frame_id(), frames_[i / kGeometryCount]);
      EXPECT_EQ(geometry.id(), geometries_[i]);
      EXPECT_EQ(geometry.child_geometry_ids().size(), 0);
      EXPECT_FALSE(geometry.parent_id());
      EXPECT_EQ(geometry.name(), geometry_names_[i]);
      EXPECT_EQ(geometry.index(), i);
      EXPECT_FALSE(geometry.render_index(kDummyRenderName));
      EXPECT_FALSE(geometry.proximity_index().is_valid());
      EXPECT_EQ(geometry.child_geometry_ids().size(), 0);

      // Note: There are no geometries parented to other geometries. The results
      // of GetPoseInFrame() and GetPoseInParent() must be the identical (as
      // the documentation for GeometryState::GetPoseInParent() indicates).
      EXPECT_TRUE(CompareMatrices(
          geometry_state_.GetPoseInFrame(geometry.id()).matrix(),
          X_FGs_[i].matrix()));
      EXPECT_TRUE(CompareMatrices(
          geometry_state_.GetPoseInParent(geometry.id()).matrix(),
          X_FGs_[i].matrix()));

      EXPECT_EQ(
          gs_tester_.get_geometry_index_id_map()[geometry.index()],
          geometry.id());
    }
  }
  EXPECT_EQ(static_cast<int>(gs_tester_.get_geometry_world_poses().size()),
            single_tree_total_geometry_count());
  EXPECT_EQ(gs_tester_.get_frame_parent_poses().size(), kFrameCount + 1);
}

// Tests the GetNum*Geometry*Methods.
TEST_F(GeometryStateTest, GetNumGeometryTests) {
  SetUpSingleSourceTree(Assign::kProximity);

  EXPECT_EQ(single_tree_total_geometry_count(),
            geometry_state_.get_num_geometries());
  EXPECT_EQ(single_tree_total_geometry_count(),
            geometry_state_.GetNumGeometriesWithRole(Role::kProximity));
  EXPECT_EQ(0, geometry_state_.GetNumGeometriesWithRole(Role::kPerception));
  EXPECT_EQ(0, geometry_state_.GetNumGeometriesWithRole(Role::kIllustration));

  for (int i = 0; i < kFrameCount; ++i) {
    EXPECT_EQ(kGeometryCount,
              geometry_state_.GetNumFrameGeometries(frames_[i]));
    EXPECT_EQ(kGeometryCount,
              geometry_state_.GetNumFrameGeometriesWithRole(frames_[i],
                                                            Role::kProximity));
    EXPECT_EQ(0,
              geometry_state_.GetNumFrameGeometriesWithRole(
                  frames_[i], Role::kPerception));
    EXPECT_EQ(0,
              geometry_state_.GetNumFrameGeometriesWithRole(
                  frames_[i], Role::kIllustration));
  }
}

// Tests that an attempt to add a frame to an invalid source throws an exception
// with meaningful message.
TEST_F(GeometryStateTest, AddFrameToInvalidSource) {
  const SourceId s_id = SourceId::get_new_id();  // Not a registered source.
  DRAKE_ASSERT_THROWS_MESSAGE(
      geometry_state_.RegisterFrame(s_id, *frame_), std::logic_error,
      "Referenced geometry source \\d+ is not registered.");
}

// Tests that a frame added to a valid source appears in the source's frames.
TEST_F(GeometryStateTest, AddFirstFrameToValidSource) {
  const SourceId s_id = NewSource();
  const FrameId fid = geometry_state_.RegisterFrame(s_id, *frame_);
  EXPECT_EQ(fid, frame_->id());
  EXPECT_TRUE(geometry_state_.BelongsToSource(fid, s_id));
  const auto &frame_set = geometry_state_.GetFramesForSource(s_id);
  EXPECT_NE(frame_set.find(fid), frame_set.end());
  EXPECT_EQ(frame_set.size(), 1);
  // The frame *is* a root frame; so both sets should be the same size.
  EXPECT_EQ(gs_tester_.get_source_frame_id_map().at(s_id).size(),
            gs_tester_.get_source_root_frame_map().at(s_id).size());
}

// Tests that a frame added to a valid source which already has frames is
// correctly appended.
TEST_F(GeometryStateTest, AddFrameToSourceWithFrames) {
  const SourceId s_id = SetUpSingleSourceTree();
  const FrameId fid = geometry_state_.RegisterFrame(s_id, *frame_);
  EXPECT_EQ(fid, frame_->id());
  EXPECT_TRUE(geometry_state_.BelongsToSource(fid, s_id));
  const auto &frame_set = geometry_state_.GetFramesForSource(s_id);
  EXPECT_NE(frame_set.find(fid), frame_set.end());
  EXPECT_EQ(frame_set.size(), kFrameCount + 1);
  // The added frame *is* a root frame. The single-source tree has *one*
  // non-root frame. This "off-by-one" property should be preserved.
  EXPECT_EQ(gs_tester_.get_source_frame_id_map().at(s_id).size() - 1,
            gs_tester_.get_source_root_frame_map().at(s_id).size());
}

// Tests that a frame added to a new source doesn't modify previously existing
// sources.
TEST_F(GeometryStateTest, AddFrameToNewSourceWithFrames) {
  const SourceId s_id = SetUpSingleSourceTree();
  const SourceId new_s_id = geometry_state_.RegisterNewSource("new_source");
  const FrameId fid = geometry_state_.RegisterFrame(new_s_id, *frame_);
  EXPECT_EQ(fid, frame_->id());
  // Confirm addition.
  EXPECT_TRUE(geometry_state_.BelongsToSource(fid, new_s_id));
  {
    const auto &frame_set = geometry_state_.GetFramesForSource(new_s_id);
    EXPECT_NE(frame_set.find(fid), frame_set.end());
    EXPECT_EQ(frame_set.size(), 1);
  }
  // Confirm original source is unchanged.
  {
    const auto &frame_set = geometry_state_.GetFramesForSource(s_id);
    EXPECT_EQ(frame_set.find(fid), frame_set.end());
    EXPECT_EQ(frame_set.size(), kFrameCount);
  }
}

// Tests the functionality of adding a frame to another frame.
TEST_F(GeometryStateTest, AddFrameOnFrame) {
  const SourceId s_id = SetUpSingleSourceTree();
  const FrameId fid = geometry_state_.RegisterFrame(s_id, frames_[0], *frame_);
  EXPECT_EQ(fid, frame_->id());
  // Includes the kFrameCount frames added in SetUpSingleSourceTree, the
  // frame we just added above (fid), and the world frame.
  EXPECT_EQ(geometry_state_.get_num_frames(), kFrameCount + 2);
  EXPECT_TRUE(geometry_state_.BelongsToSource(fid, s_id));

  // Test parent-child relationship wiring.
  //  Difference between total and root frames. I now have *two* non-root
  // frames.
  EXPECT_EQ(gs_tester_.get_source_frame_id_map().at(s_id).size() - 2,
            gs_tester_.get_source_root_frame_map().at(s_id).size());
  //  Frame for this id has frame_[0] as parent.
  const auto& frame = gs_tester_.get_frames().at(fid);
  EXPECT_TRUE(frame.has_parent(frames_[0]));
  //  Parent frame has this as child.
  const auto& parent = gs_tester_.get_frames().at(frames_[0]);
  EXPECT_TRUE(parent.has_child(fid));
}

// Confirms that adding two frames with the same id causes an error.
TEST_F(GeometryStateTest, AddFrameWithDuplicateId) {
  const SourceId s_id = NewSource();
  const FrameId f_id = geometry_state_.RegisterFrame(s_id, *frame_);
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterFrame(s_id, *frame_), std::logic_error,
      "Registering frame with an id that has already been registered: \\d+");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterFrame(s_id, f_id, *frame_), std::logic_error,
      "Registering frame with an id that has already been registered: \\d+");
}

// Tests the frame iterator, confirming that it iterates through all frames.
TEST_F(GeometryStateTest, FrameIdRange) {
  SetUpSingleSourceTree();
  unordered_set<FrameId> all_frames(frames_.begin(), frames_.end());
  for (FrameId id : geometry_state_.get_frame_ids()) {
    // This should remove exactly one element. The world frame is *not* stored
    // in frames_.
    if (id != InternalFrame::world_frame_id()) {
      EXPECT_EQ(all_frames.erase(id), 1);
    }
  }
  // There shouldn't be any left over.
  EXPECT_EQ(all_frames.size(), 0);
}

// Tests registration of geometry on valid source and frame. This relies on the
// correctness of GeometryState::BelongsToSource(GeometryId, SourceId) and
// GeometryState::GetFrameId(GeometryId) and, therefore, implicitly tests them.
TEST_F(GeometryStateTest, RegisterGeometryGoodSource) {
  const SourceId s_id = NewSource();
  const FrameId f_id = geometry_state_.RegisterFrame(s_id, *frame_);
  const GeometryId expected_g_id = instance_->id();
  const GeometryId g_id =
      geometry_state_.RegisterGeometry(s_id, f_id, move(instance_));
  EXPECT_EQ(g_id, expected_g_id);
  EXPECT_EQ(geometry_state_.GetFrameId(g_id), f_id);
  EXPECT_TRUE(geometry_state_.BelongsToSource(g_id, s_id));
  const Isometry3d& X_FG = geometry_state_.GetPoseInFrame(g_id);
  EXPECT_TRUE(CompareMatrices(X_FG.matrix(), instance_pose_.matrix()));

  EXPECT_TRUE(gs_tester_.get_frames().at(f_id).has_child(g_id));
  const auto& geometry = gs_tester_.get_geometries().at(g_id);
  EXPECT_TRUE(geometry.is_child_of_frame(f_id));
  EXPECT_FALSE(geometry.parent_id());
}

// Confirms that registering two geometries with the same id causes failure.
TEST_F(GeometryStateTest, RegisterDuplicateGeometry) {
  const SourceId s_id = NewSource();
  const FrameId f_id = geometry_state_.RegisterFrame(s_id, *frame_);
  auto instance_copy = make_unique<GeometryInstance>(*instance_);
  geometry_state_.RegisterGeometry(s_id, f_id, move(instance_));
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterGeometry(s_id, f_id, move(instance_copy)),
      std::logic_error,
      "Registering geometry with an id that has already been registered: \\d+");
}

// Tests registration of geometry on invalid source.
TEST_F(GeometryStateTest, RegisterGeometryMissingSource) {
  const SourceId s_id = SourceId::get_new_id();
  const FrameId f_id = FrameId::get_new_id();
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterGeometry(s_id, f_id, move(instance_)),
      std::logic_error, "Referenced geometry source \\d+ is not registered.");
}

// Tests registration of geometry on valid source and non-existent frame.
TEST_F(GeometryStateTest, RegisterGeometryMissingFrame) {
  const SourceId s_id = NewSource();
  const FrameId f_id = FrameId::get_new_id();
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterGeometry(s_id, f_id, move(instance_)),
      std::logic_error,
      "Referenced frame \\d+ for source \\d+\\,"
      " but the frame doesn't belong to the source.");
}

// Tests error resulting from passing a null GeometryInstance.
TEST_F(GeometryStateTest, RegisterNullGeometry) {
  const SourceId s_id = NewSource();
  const FrameId f_id = geometry_state_.RegisterFrame(s_id, *frame_);
  unique_ptr<GeometryInstance> null_geometry;
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterGeometry(s_id, f_id, move(null_geometry)),
      std::logic_error,
      "Registering null geometry to frame \\d+, on source \\d+.");
}

// Tests the logic for hanging a geometry on another geometry. This confirms
// topology and pose values.
TEST_F(GeometryStateTest, RegisterGeometryonValidGeometry) {
  const SourceId s_id = SetUpSingleSourceTree();
  const double x = 3;
  const double y = 2;
  const double z = 1;
  Isometry3d pose{Translation3d{x, y, z}};
  const int parent_index = 0;
  const GeometryId parent_id = geometries_[parent_index];
  const FrameId frame_id = geometry_state_.GetFrameId(parent_id);
  auto instance = make_unique<GeometryInstance>(
      pose, make_unique<Sphere>(1), "sphere");
  GeometryId expected_g_id = instance->id();
  GeometryId g_id =
      geometry_state_.RegisterGeometryWithParent(s_id,
                                                 parent_id,
                                                 move(instance));
  EXPECT_EQ(g_id, expected_g_id);

  // This relies on the gᵗʰ geometry having position [ g+1 0 0 ]ᵀ. The parent
  // geometry is at [parent_index + 1, 0, 0] and this is at [3, 2, 1]. They
  // simply sum up. The parent has *no* rotation so the resultant transform is
  // simply the sum of the translation vectors.
  const Isometry3d expected_pose_in_frame{
      Translation3d{(parent_index + 1) + x, y, z}};
  EXPECT_EQ(frame_id, geometry_state_.GetFrameId(g_id));

  const Isometry3d& X_FG = geometry_state_.GetPoseInFrame(g_id);
  EXPECT_TRUE(CompareMatrices(X_FG.matrix(), expected_pose_in_frame.matrix(),
                  1e-14, MatrixCompareType::absolute));
  const Isometry3d& X_PG = geometry_state_.GetPoseInParent(g_id);
  EXPECT_TRUE(CompareMatrices(X_PG.matrix(), pose.matrix(),
                  1e-14, MatrixCompareType::absolute));

  EXPECT_TRUE(gs_tester_.get_frames().at(frame_id).has_child(g_id));
  const auto& geometry = gs_tester_.get_geometries().at(g_id);
  EXPECT_EQ(geometry.frame_id(), frame_id);
  EXPECT_TRUE(geometry.is_child_of_geometry(parent_id));
  EXPECT_TRUE(gs_tester_.get_geometries().at(parent_id).has_child(g_id));

  const InternalGeometry* parent = gs_tester_.GetGeometry(parent_id);
  const InternalGeometry* child = gs_tester_.GetGeometry(g_id);
  EXPECT_TRUE(parent->has_child(g_id));
  EXPECT_TRUE(child->parent_id());  // implicit cast of optional --> bool.
  EXPECT_EQ(parent_id, *child->parent_id());
}

// Tests the response to the erroneous action of trying to hang a new geometry
// on a non-existent geometry id.
TEST_F(GeometryStateTest, RegisterGeometryonInvalidGeometry) {
  const SourceId s_id = SetUpSingleSourceTree();
  auto instance = make_unique<GeometryInstance>(
      Isometry3d::Identity(), make_unique<Sphere>(1), "sphere");
  const GeometryId junk_id = GeometryId::get_new_id();
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterGeometryWithParent(s_id, junk_id, move(instance)),
      std::logic_error,
      "Referenced geometry \\d+ has not been registered.");
}

// Tests the response to passing a null pointer as a GeometryInstance.
TEST_F(GeometryStateTest, RegisterNullGeometryonGeometry) {
  const SourceId s_id = SetUpSingleSourceTree();
  unique_ptr<GeometryInstance> instance;
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterGeometryWithParent(s_id, geometries_[0],
                                                 move(instance)),
      std::logic_error,
      "Registering null geometry to geometry \\d+, on source \\d+.");
}

// Tests the registration of anchored geometry.
TEST_F(GeometryStateTest, RegisterAnchoredGeometry) {
  const SourceId s_id = NewSource("new source");
  auto instance = make_unique<GeometryInstance>(
      Isometry3d::Identity(), make_unique<Sphere>(1), "sphere");
  const GeometryId expected_g_id = instance->id();
  const auto g_id =
      geometry_state_.RegisterAnchoredGeometry(s_id, move(instance));
  EXPECT_EQ(g_id, expected_g_id);
  EXPECT_TRUE(geometry_state_.BelongsToSource(g_id, s_id));
  const InternalGeometry* g = gs_tester_.GetGeometry(g_id);
  EXPECT_NE(g, nullptr);
  EXPECT_EQ(InternalFrame::world_frame_id(), g->frame_id());
  EXPECT_EQ(s_id, g->source_id());
  // Assigned directly to the world frame means the pose in parent and frame
  // should match.
  EXPECT_TRUE(CompareMatrices(g->X_FG().matrix(), g->X_PG().matrix()));
}

// Tests the registration of a new geometry on another geometry.
TEST_F(GeometryStateTest, RegisterAnchoredOnAnchoredGeometry) {
  // Add an anchored geometry.
  const SourceId s_id = NewSource("new source");
  Isometry3d pose{Translation3d{1, 2, 3}};
  auto instance = make_unique<GeometryInstance>(
      pose, make_unique<Sphere>(1), "sphere1");
  auto parent_id = geometry_state_.RegisterAnchoredGeometry(s_id,
                                                            move(instance));

  pose = Isometry3d::Identity();
  instance = make_unique<GeometryInstance>(
      pose, make_unique<Sphere>(1), "sphere2");
  auto child_id = geometry_state_.RegisterGeometryWithParent(s_id, parent_id,
                                                             move(instance));
  const InternalGeometry* parent = gs_tester_.GetGeometry(parent_id);
  const InternalGeometry* child = gs_tester_.GetGeometry(child_id);
  EXPECT_TRUE(parent->has_child(child_id));
  EXPECT_TRUE(static_cast<bool>(child->parent_id()));
  EXPECT_EQ(parent_id, *child->parent_id());
  EXPECT_TRUE(CompareMatrices(pose.matrix(), child->X_PG().matrix()));
  const Isometry3d& X_FP = parent->X_FG();
  EXPECT_TRUE(CompareMatrices((X_FP * pose).matrix(), child->X_FG().matrix()));
  EXPECT_EQ(InternalFrame::world_frame_id(), parent->frame_id());
}

// Confirms that registering two geometries with the same id causes failure.
TEST_F(GeometryStateTest, RegisterDuplicateAnchoredGeometry) {
  const SourceId s_id = NewSource();
  auto instance_copy = make_unique<GeometryInstance>(*instance_);
  geometry_state_.RegisterAnchoredGeometry(s_id, move(instance_));
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterAnchoredGeometry(s_id, move(instance_copy)),
      std::logic_error,
      "Registering geometry with an id that has already been "
      "registered: \\d+");
}

// Tests the attempt to register anchored geometry on an invalid source.
TEST_F(GeometryStateTest, RegisterAnchoredGeometryInvalidSource) {
  auto instance = make_unique<GeometryInstance>(
      Isometry3d::Identity(), make_unique<Sphere>(1), "sphere");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterAnchoredGeometry(SourceId::get_new_id(),
                                               move(instance)),
      std::logic_error,
      "Referenced geometry source \\d+ is not registered.");
}

// Tests the response of attempting to register a null pointer GeometryInstance
// as anchored geometry.
TEST_F(GeometryStateTest, RegisterAnchoredNullGeometry) {
  unique_ptr<GeometryInstance> instance;
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterAnchoredGeometry(SourceId::get_new_id(),
                                               move(instance)),
      std::logic_error,
      "Registering null geometry to frame \\d+, on source \\d+.");
}

// Tests the RemoveGeometry() functionality. This action will have several
// effects which will all be confirmed. By removing geometry 0 (the first
// registered geometry and first dynamic geometry), the following effects are
// expected:
//   1. GeometryState moves geometries around to maintain compact distribution
//      of GeometryIndex values. Therefore, the GeometryIndex(0) (which
//      previously referred to the removed geometry now refers to the anchored
//      geometry (the *last* geometry added to the state in the tree).
//   2. The proximity engine will likewise shuffle indices. Therefore,
//      ProximityIndex(0) will now refer to the last _dynamic_ geometry (the
//      dynamic geometry that previously had the highest ProximityIndex value
//      because the proximity engine only moves dynamic geometries in place of
//      dynamic geometries and anchored for anchored).
//   3. The renderer engine will also shuffle indices. So, RenderIndex(0) will
//      belong to the anchored geometry. (Note: the mapping _from_ render index
//      _to_ GeometryIndex is stored inside the RenderEngine and confirming that
//      that has been correctly remapped is a test of the RenderEngine
//      implementation.)
// TODO(SeanCurtis-TRI): Consider breaking this test apart into those data
// members *owned* by GeometryState and those that are role-specific. Right now
// the test is huge and hard to follow.
TEST_F(GeometryStateTest, RemoveGeometry) {
  // Every geometry gets proximity and perception properties.
  const SourceId s_id =
      SetUpSingleSourceTree(Assign::kProximity | Assign::kPerception);

  // Set the render index to be reported as moved as the last geometry added:
  // the anchored geometry.
  render_engine_->set_moved_index(
      gs_tester_.GetGeometry(anchored_geometry_)
          ->render_index(kDummyRenderName));

  // Confirm that the render indices are set up as expected, that
  // GeometryIndex(i) has RenderIndex(i);
  for (GeometryId id : all_geometry_ids()) {
    const InternalGeometry* geometry = gs_tester_.GetGeometry(id);
    optional<RenderIndex> render_index =
        geometry->render_index(kDummyRenderName);
    ASSERT_TRUE(render_index);
    int geometry_index_value = geometry->index();
    EXPECT_EQ(*render_index, geometry_index_value);
  }

  // Pose all of the frames to the default poses (X_PFs_).
  FramePoseVector<double> poses;
  for (int f = 0; f < static_cast<int>(frames_.size()); ++f) {
    poses.set_value(frames_[f], X_PFs_[f]);
  }
  gs_tester_.SetFramePoses(s_id, poses);
  gs_tester_.FinalizePoseUpdate();

  // The geometry to remove, its parent frame, and its engine indices.
  const GeometryId g_id = geometries_[0];
  const FrameId f_id = frames_[0];
  const ProximityIndex proximity_index =
      gs_tester_.get_geometries().at(g_id).proximity_index();
  const RenderIndex render_index =
      *gs_tester_.get_geometries().at(g_id).render_index(kDummyRenderName);

  // Confirm initial state.
  ASSERT_EQ(geometry_state_.GetFrameId(g_id), f_id);
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            single_tree_total_geometry_count());
  EXPECT_NE(
      gs_tester_.get_geometries().at(geometries_.back()).proximity_index(),
      proximity_index);
  EXPECT_NE(*gs_tester_.get_geometries()
                 .at(geometries_.back())
                 .render_index(kDummyRenderName),
            render_index);
  EXPECT_EQ(geometry_state_.GetNumDynamicGeometries(),
            single_tree_dynamic_geometry_count());

  geometry_state_.RemoveGeometry(s_id, g_id);

  EXPECT_EQ(geometry_state_.get_num_geometries(),
            single_tree_total_geometry_count() - 1);
  EXPECT_EQ(geometry_state_.GetNumDynamicGeometries(),
            single_tree_dynamic_geometry_count() - 1);

  EXPECT_FALSE(gs_tester_.get_frames().at(f_id).has_child(g_id));
  EXPECT_EQ(gs_tester_.get_geometries().count(g_id), 0);

  // Confirm GeometryIndex(0) now maps to the anchored geometry.
  const InternalGeometry& anchored_geometry =
      gs_tester_.get_geometries().at(anchored_geometry_);
  EXPECT_EQ(anchored_geometry.proximity_index(), proximity_index);
  EXPECT_EQ(gs_tester_.get_geometry_index_id_map()[0], anchored_geometry_);

  // Confirm that the anchored geometry now contains RenderIndex(0) for the
  // dummy render engine.
  EXPECT_EQ(*anchored_geometry.render_index(kDummyRenderName), RenderIndex(0));

  // Confirm that the anchored geometry now contains ProximityIndex(0) --
  // also confirm that the X_WG_ quantity indexed by proximity index points to
  // that geometry's world pose.
  EXPECT_EQ(
      gs_tester_.get_geometries().at(geometries_.back()).proximity_index(),
      proximity_index);
  // The anchored geometry's world pose is simply the last pose in X_FGs_;
  // it was added last and its parent is the world frame.
  const Isometry3d& X_WA = X_FGs_.back();
  EXPECT_TRUE(CompareMatrices(gs_tester_.get_geometry_world_poses()[0].matrix(),
                              X_WA.matrix()));

  // Confirm that, post removal, updating poses still works.
  EXPECT_NO_THROW(gs_tester_.FinalizePoseUpdate());

  // Adding a new geometry should bring the number of total geometries back to
  // the original count.
  const GeometryId added_id = geometry_state_.RegisterGeometry(
      source_id_, frames_[0],
      make_unique<GeometryInstance>(Isometry3d::Identity(),
                                    make_unique<Sphere>(1), "newest"));
  const InternalGeometry& added_geo = gs_tester_.get_geometries().at(added_id);
  // Highest index should be number of geometries - 1.
  EXPECT_EQ(added_geo.index(),
            GeometryIndex(single_tree_total_geometry_count() - 1));

  // Adding proximity role to the new geometry brings the total number of
  // dynamic geometries with proximity roles back up to the original value.
  geometry_state_.AssignRole(source_id_, added_id, ProximityProperties());
  // As the added geometry is dynamic, the highest index is total number of
  // _dynamic_ geometries - 1.
  EXPECT_EQ(added_geo.proximity_index(),
            ProximityIndex(single_tree_dynamic_geometry_count() - 1));

  // Now remove the *final* geometry; even though it doesn't require re-ordering
  // proximity indices, it should still keep things valid -- in other words,
  // the mapping from proximity index to internal index for *dynamic* geometries
  // should shrink appropriately.
  geometry_state_.RemoveGeometry(s_id, added_id);
  // Confirm that, post removal, updating poses still works.
  EXPECT_NO_THROW(gs_tester_.FinalizePoseUpdate());

  // Test the case where a geometry gets re-ordered, but it has no roles. To
  // make sure the geometry moves, it needs to _not_ be the last added geometry.
  // So, we add the geometry we're going to remove _and_ an additional geometry
  // after it.
  const GeometryId no_role_id = geometry_state_.RegisterGeometry(
      source_id_, frames_[0],
      make_unique<GeometryInstance>(
          Isometry3d::Identity(), make_unique<Sphere>(1), "no_role_geometry"));
  geometry_state_.RegisterGeometry(
      source_id_, frames_[0],
      make_unique<GeometryInstance>(
          Isometry3d::Identity(), make_unique<Sphere>(1), "no_role_geometry2"));
  EXPECT_NO_THROW(geometry_state_.RemoveGeometry(s_id, no_role_id));
  EXPECT_NO_THROW(gs_tester_.FinalizePoseUpdate());
}

// Tests the RemoveGeometry functionality in which the geometry removed has
// geometry children.
TEST_F(GeometryStateTest, RemoveGeometryTree) {
  const SourceId s_id = SetUpSingleSourceTree(Assign::kProximity);
  // Pose all of the frames to the specified poses in their parent frame.
  FramePoseVector<double> poses;
  for (int f = 0; f < static_cast<int>(frames_.size()); ++f) {
    poses.set_value(frames_[f], X_PFs_[f]);
  }
  gs_tester_.SetFramePoses(s_id, poses);
  gs_tester_.FinalizePoseUpdate();

  // The geometry to remove, its parent frame, and its proximity index.
  const GeometryId root_id = geometries_[0];
  const FrameId f_id = frames_[0];
  auto proximity_index =
      gs_tester_.get_geometries().at(root_id).proximity_index();
  // Confirm that the first geometry belongs to the first frame.
  ASSERT_EQ(geometry_state_.GetFrameId(root_id), f_id);
  // Hang geometry from the first geometry.
  const GeometryId g_id = geometry_state_.RegisterGeometryWithParent(
      s_id, root_id,
      make_unique<GeometryInstance>(Isometry3d::Identity(),
                                    unique_ptr<Shape>(new Sphere(1)), "leaf"));
  geometry_state_.AssignRole(s_id, g_id, ProximityProperties());

  EXPECT_EQ(geometry_state_.get_num_geometries(),
            single_tree_total_geometry_count() + 1);
  EXPECT_EQ(geometry_state_.GetNumDynamicGeometries(),
            single_tree_dynamic_geometry_count() + 1);
  EXPECT_EQ(geometry_state_.GetFrameId(g_id), f_id);
  EXPECT_EQ(gs_tester_.get_geometries().at(g_id).proximity_index(),
            geometries_.size());

  geometry_state_.RemoveGeometry(s_id, root_id);
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            single_tree_total_geometry_count() - 1);
  EXPECT_EQ(geometry_state_.GetNumDynamicGeometries(),
            single_tree_dynamic_geometry_count() - 1);

  const auto& frame = gs_tester_.get_frames().at(f_id);
  EXPECT_FALSE(frame.has_child(root_id));
  EXPECT_FALSE(frame.has_child(g_id));
  EXPECT_EQ(gs_tester_.get_geometries().count(root_id), 0);
  EXPECT_EQ(gs_tester_.get_geometries().count(g_id), 0);

  // Deleting a tree of geometry is a bottom-up operation. That means the newly
  // added leaf geometry will be deleted first. By construction, it *is* the
  // last geometry added (highest proximity index and highest geometry index)
  // so no index swapping will take place to remove it. However, there will be
  // swapping akin to that in the RemoveGeometry() test.

  // Confirm GeometryIndex(0) now maps to the anchored geometry.
  const GeometryId last_geometry_id = anchored_geometry_;
  const auto& last_geometry =
      gs_tester_.get_geometries().at(last_geometry_id);
  EXPECT_EQ(last_geometry.proximity_index(), proximity_index);
  EXPECT_EQ(gs_tester_.get_geometry_index_id_map()[0], last_geometry_id);

  // Confirm that ProximityIndex(0) belongs to the last dynamic geometry --
  // also confirm that the X_WG_ quantity indexed by proximity index points to
  // that geometry's world pose.
  EXPECT_EQ(
      gs_tester_.get_geometries().at(geometries_.back()).proximity_index(),
      proximity_index);
  const Isometry3d X_WG = X_WFs_.back() * X_FGs_.back();
  EXPECT_TRUE(CompareMatrices(gs_tester_.get_geometry_world_poses()[0].matrix(),
                              X_WG.matrix()));
}

// Tests the RemoveGeometry functionality in which the geometry is a child of
// another geometry (and has no child geometries itself).
TEST_F(GeometryStateTest, RemoveChildLeaf) {
  const SourceId s_id = SetUpSingleSourceTree(Assign::kProximity);
  // The geometry parent and frame to which it belongs.
  const GeometryId parent_id = geometries_[0];
  const FrameId frame_id = frames_[0];
  // Confirm that the first geometry belongs to the first frame.
  ASSERT_EQ(geometry_state_.GetFrameId(parent_id), frame_id);
  // Hang geometry from the first geometry.
  const GeometryId g_id = geometry_state_.RegisterGeometryWithParent(
      s_id, parent_id,
      make_unique<GeometryInstance>(Isometry3d::Identity(),
                                    unique_ptr<Shape>(new Sphere(1)), "leaf"));
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            single_tree_total_geometry_count() + 1);
  EXPECT_EQ(geometry_state_.GetFrameId(g_id), frame_id);

  geometry_state_.RemoveGeometry(s_id, g_id);

  EXPECT_EQ(geometry_state_.get_num_geometries(),
            single_tree_total_geometry_count());
  EXPECT_EQ(geometry_state_.GetNumDynamicGeometries(), geometries_.size());
  EXPECT_EQ(geometry_state_.GetFrameId(parent_id), frame_id);

  EXPECT_FALSE(gs_tester_.get_frames().at(frame_id).has_child(g_id));
  EXPECT_TRUE(gs_tester_.get_frames().at(frame_id).has_child(parent_id));
  EXPECT_FALSE(gs_tester_.get_geometries().at(parent_id).has_child(g_id));

  // The geometry we deleted is the *last*; the engine indices of all other
  // geometries should be unchanged.
  for (size_t i = 0; i < geometries_.size(); ++i) {
    EXPECT_EQ(gs_tester_.get_geometry_index_id_map().at(i),
              geometries_[i]);
  }
}

// Tests the response to invalid use of RemoveGeometry.
TEST_F(GeometryStateTest, RemoveGeometryInvalid) {
  const SourceId s_id = SetUpSingleSourceTree();

  // Case: Invalid source id, valid geometry id.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveGeometry(SourceId::get_new_id(),
                                     geometries_[0]),
      std::logic_error,
      "Referenced geometry source \\d+ is not registered.");

  // Case: Invalid geometry id, valid source id.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveGeometry(s_id, GeometryId::get_new_id()),
      std::logic_error,
      "Referenced geometry \\d+ has not been registered.");

  // Case: Valid geometry and source, but geometry belongs to different source.
  const SourceId s_id2 = geometry_state_.RegisterNewSource("new_source");
  const FrameId frame_id = geometry_state_.RegisterFrame(s_id2, *frame_);
  EXPECT_EQ(geometry_state_.get_num_frames(), single_tree_frame_count() + 1);
  const GeometryId g_id = geometry_state_.RegisterGeometry(
      s_id2, frame_id,
      make_unique<GeometryInstance>(Isometry3d::Identity(),
                                    unique_ptr<Shape>(new Sphere(1)), "new"));
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            single_tree_total_geometry_count() + 1);
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveGeometry(s_id, g_id),
      std::logic_error,
      "Trying to remove geometry \\d+ from source \\d+.+geometry doesn't "
          "belong.+");
}

// Tests removal of anchored geometry.
TEST_F(GeometryStateTest, RemoveAnchoredGeometry) {
  const SourceId s_id = SetUpSingleSourceTree(Assign::kProximity);

  const Vector3<double> normal{0, 1, 0};
  const Vector3<double> point{1, 1, 1};
  const auto anchored_id_1 = geometry_state_.RegisterAnchoredGeometry(
      s_id,
      make_unique<GeometryInstance>(HalfSpace::MakePose(normal, point),
                                    make_unique<HalfSpace>(), "anchored1"));
  geometry_state_.AssignRole(s_id, anchored_id_1, ProximityProperties());
  // Confirm conditions of having added the anchored geometry.
  EXPECT_EQ(gs_tester_.get_geometry_index_id_map().size(),
            single_tree_total_geometry_count() + 1);
  EXPECT_TRUE(geometry_state_.BelongsToSource(anchored_id_1, s_id));
  EXPECT_EQ(gs_tester_.GetGeometry(anchored_id_1)->proximity_index(), 1);

  geometry_state_.RemoveGeometry(s_id, anchored_geometry_);

  EXPECT_EQ(gs_tester_.GetGeometry(anchored_geometry_), nullptr);

  // Two indices move:
  //  The new anchored geometry gets the geometry index of the old anchored
  //     geometry.
  //  The new anchored geometry engine index goes down to zero.
  EXPECT_EQ(gs_tester_.get_geometry_index_id_map().size(),
            single_tree_total_geometry_count());
  EXPECT_EQ(gs_tester_.get_geometry_index_id_map().back(),
            anchored_id_1);
  // The highest *index* is always count - 1.
  EXPECT_EQ(gs_tester_.get_geometries().at(anchored_id_1).index(),
            single_tree_total_geometry_count() - 1);
  EXPECT_EQ(gs_tester_.get_geometries().at(anchored_id_1).proximity_index(), 0);
}

// Tests removal of geometry when collision filters are present. As geometries
// move around, their filter semantics should follow.
TEST_F(GeometryStateTest, RemoveGeometryWithCollisionFilters) {
  const SourceId s_id = SetUpSingleSourceTree(Assign::kProximity);
  auto is_filtered = [this](int index0, int index1) {
    return geometry_state_.CollisionFiltered(geometries_[index0],
                                             geometries_[index1]);
  };

  // Collision filters applied to siblings.
  EXPECT_TRUE(is_filtered(0, 1));
  EXPECT_TRUE(is_filtered(2, 3));
  EXPECT_TRUE(is_filtered(4, 5));
  // All other pairs are unfiltered.
  EXPECT_FALSE(is_filtered(0, 2));
  EXPECT_FALSE(is_filtered(0, 3));
  EXPECT_FALSE(is_filtered(0, 4));
  EXPECT_FALSE(is_filtered(0, 5));
  EXPECT_FALSE(is_filtered(1, 2));
  EXPECT_FALSE(is_filtered(1, 3));
  EXPECT_FALSE(is_filtered(1, 4));
  EXPECT_FALSE(is_filtered(1, 5));
  EXPECT_FALSE(is_filtered(2, 4));
  EXPECT_FALSE(is_filtered(2, 5));
  EXPECT_FALSE(is_filtered(3, 4));
  EXPECT_FALSE(is_filtered(3, 5));

  // Note: removing anchored geometry because it was the last registered; it
  // will be moved but has no filtered relationships with anyone. So, pop it
  // and move on.
  geometry_state_.RemoveGeometry(s_id, anchored_geometry_);
  // Now geometries_[5] is the last; it will be moved into GeometryIndex(1). The
  // failure we're testing for is that it doesn't inherit geometries_[1]'s
  // collision filter semantics, but maintains its own.
  geometry_state_.RemoveGeometry(s_id, geometries_[1]);

  // Repeat the test above, but exclude all pairs including the removed
  // geometry.

  // Collision filters applied to siblings.
  EXPECT_TRUE(is_filtered(2, 3));
  EXPECT_TRUE(is_filtered(4, 5));
  // All other pairs are unfiltered.
  EXPECT_FALSE(is_filtered(0, 2));
  EXPECT_FALSE(is_filtered(0, 3));
  EXPECT_FALSE(is_filtered(0, 4));
  EXPECT_FALSE(is_filtered(0, 5));
  EXPECT_FALSE(is_filtered(2, 4));
  EXPECT_FALSE(is_filtered(2, 5));
  EXPECT_FALSE(is_filtered(3, 4));
  EXPECT_FALSE(is_filtered(3, 5));
}

// Confirms the behavior for requesting geometry poses with a bad geometry
// identifier. The basic behavior is tested implicitly in other tests because
// they rely on them to validate state.
TEST_F(GeometryStateTest, GetPoseForBadGeometryId) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.GetPoseInFrame(GeometryId::get_new_id()),
      std::logic_error,
      "Referenced geometry \\d+ has not been registered.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.GetPoseInParent(GeometryId::get_new_id()),
      std::logic_error,
      "Referenced geometry \\d+ has not been registered.");
}

// This tests the source ownership functionality - a function which reports if
// a geometry or frame belongs to the specified source - in the case where the
// source id is invalid. Whether or not the frame/geometry ids are valid, the
// bad source should dominate.
TEST_F(GeometryStateTest, SourceOwnershipInvalidSource) {
  const SourceId source_id = SourceId::get_new_id();
  // Invalid frame/geometry ids.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.BelongsToSource(FrameId::get_new_id(), source_id),
      std::logic_error, "Referenced geometry source \\d+ is not registered.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.BelongsToSource(GeometryId::get_new_id(), source_id),
      std::logic_error, "Referenced geometry source \\d+ is not registered.");
  SetUpSingleSourceTree();
  const GeometryId anchored_id = geometry_state_.RegisterAnchoredGeometry(
      source_id_,
      make_unique<GeometryInstance>(Isometry3d::Identity(),
                                    make_unique<Sphere>(1),
                                    "sphere"));
  // Valid frame/geometry ids.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.BelongsToSource(frames_[0], source_id), std::logic_error,
      "Referenced geometry source \\d+ is not registered.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.BelongsToSource(geometries_[0], source_id),
      std::logic_error, "Referenced geometry source \\d+ is not registered.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.BelongsToSource(anchored_id, source_id), std::logic_error,
      "Referenced geometry source \\d+ is not registered.");
}

// This tests the source ownership functionality for frames - a function which
// reports if a frame belongs to the specified source.
TEST_F(GeometryStateTest, SourceOwnershipFrameId) {
  const SourceId s_id = SetUpSingleSourceTree();
  // Test for invalid frame.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.BelongsToSource(FrameId::get_new_id(), s_id),
      std::logic_error, "Referenced frame \\d+ has not been registered.");
  // Test for valid frame.
  EXPECT_TRUE(geometry_state_.BelongsToSource(frames_[0], s_id));
}

// This tests the source ownership functionality for geometry - a function which
// reports if a geometry belongs to the specified source. It examines dynamic
// and anchored geometry.
TEST_F(GeometryStateTest, SourceOwnershipGeometryId) {
  const SourceId s_id = SetUpSingleSourceTree();
  const GeometryId anchored_id = geometry_state_.RegisterAnchoredGeometry(
      s_id, make_unique<GeometryInstance>(Isometry3d::Identity(),
                                          make_unique<Sphere>(1),
                                          "sphere"));
  // Test for invalid geometry.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.BelongsToSource(GeometryId::get_new_id(), s_id),
      std::logic_error, "Referenced geometry \\d+ has not been registered.");
  // Test for valid geometry.
  EXPECT_TRUE(geometry_state_.BelongsToSource(geometries_[0], s_id));
  EXPECT_TRUE(geometry_state_.BelongsToSource(anchored_id, s_id));
}

// This confirms the failure state of calling GeometryState::GetFrameId with a
// bad geometry identifier.
TEST_F(GeometryStateTest, GetFrameIdFromBadId) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.GetFrameId(GeometryId::get_new_id()), std::logic_error,
      "Referenced geometry \\d+ has not been registered.");
}

// Tests the validation of the ids provided in a frame kinematics vector.
TEST_F(GeometryStateTest, ValidateFrameIds) {
  const SourceId s_id = SetUpSingleSourceTree();
  FramePoseVector<double> frame_set;
  for (auto frame_id : frames_) {
    frame_set.set_value(frame_id, Isometry3d::Identity());
  }
  // Case: frame ids are valid.
  EXPECT_NO_THROW(gs_tester_.ValidateFrameIds(s_id, frame_set));

  // Case: Right number, wrong frames.
  FramePoseVector<double> frame_set_2;
  for (int i = 0; i < kFrameCount; ++i) {
    frame_set_2.set_value(FrameId::get_new_id(), Isometry3d::Identity());
  }
  DRAKE_EXPECT_THROWS_MESSAGE(
      gs_tester_.ValidateFrameIds(s_id, frame_set_2), std::runtime_error,
      "Registered frame id \\(\\d+\\) belonging to source \\d+ was not found "
          "in the provided kinematics data.");

  // Case: Too few frames.
  FramePoseVector<double> frame_set_3;
  for (int i = 0; i < kFrameCount - 1; ++i) {
    frame_set.set_value(frames_[i], Isometry3d::Identity());
  }
  DRAKE_EXPECT_THROWS_MESSAGE(
      gs_tester_.ValidateFrameIds(s_id, frame_set_3), std::runtime_error,
      "Disagreement in expected number of frames \\(\\d+\\)"
      " and the given number of frames \\(\\d+\\).");
}

// Tests the GeometryState::SetFramePoses() method. This doesn't test
// invalid kinematics sets (as that has been tested already). It simply confirms
// that for valid values, the geometries are posed as expected in the world
// frame. This only tests pose (not velocity or acceleration). These tests use
// simple transforms because it isn't validating matrix multiplication, only
// that the right matrix multiplications are performed based on the hierarchy
// of constructs.
TEST_F(GeometryStateTest, SetFramePoses) {
  const SourceId s_id = SetUpSingleSourceTree();
  // A vector of poses we will use to populate FramePoseVectors.
  vector<Isometry3d> frame_poses;
  for (int i = 0; i < kFrameCount; ++i) {
    frame_poses.push_back(Isometry3d::Identity());
  }

  auto make_pose_vector =
      [&frame_poses, this]() -> FramePoseVector<double> {
    const int count = static_cast<int>(this->frames_.size());
    FramePoseVector<double> poses;
    for (int i = 0; i < count; ++i) {
      poses.set_value(this->frames_[i], frame_poses[i]);
    }
    return poses;
  };

  // NOTE: Don't re-order the tests; they rely on an accumulative set of changes
  // to the `frame_poses` vector of poses.

  // Case 1: Set all frames to identity poses. The world pose of all the
  // geometry should be that of the geometry in its frame.
  FramePoseVector<double> poses1 = make_pose_vector();
  gs_tester_.SetFramePoses(s_id, poses1);
  const auto& world_poses = gs_tester_.get_geometry_world_poses();
  for (int i = 0; i < kFrameCount * kGeometryCount; ++i) {
    EXPECT_TRUE(CompareMatrices(world_poses[i].matrix().block<3, 4>(0, 0),
                                X_FGs_[i].matrix().block<3, 4>(0, 0)));
  }

  // Case 2: Move the two *root* frames 1 unit in the +y direction. The f2 will
  // stay at the identity.
  // The final geometry poses should all be offset by 1 unit in the y.
  const Isometry3d offset{Translation3d{0, 1, 0}};
  frame_poses[0] = offset;
  frame_poses[1] = offset;
  FramePoseVector<double> poses2 = make_pose_vector();
  gs_tester_.SetFramePoses(s_id, poses2);
  for (int i = 0; i < kFrameCount * kGeometryCount; ++i) {
    EXPECT_TRUE(
        CompareMatrices(world_poses[i].matrix().block<3, 4>(0, 0),
                        (offset * X_FGs_[i].matrix()).block<3, 4>(0, 0)));
  }

  // Case 3: All frames get set to move up one unit. This will leave geometries
  // 0, 1, 2, & 3 moved up 1, and geometries 4 & 5 moved up two.
  frame_poses[2] = offset;
  FramePoseVector<double> poses3 = make_pose_vector();
  gs_tester_.SetFramePoses(s_id, poses3);
  for (int i = 0; i < (kFrameCount - 1) * kGeometryCount; ++i) {
    EXPECT_TRUE(
        CompareMatrices(world_poses[i].matrix().block<3, 4>(0, 0),
                        (offset * X_FGs_[i].matrix()).block<3, 4>(0, 0)));
  }
  for (int i = (kFrameCount - 1) * kGeometryCount;
       i < kFrameCount * kGeometryCount; ++i) {
    EXPECT_TRUE(CompareMatrices(
        world_poses[i].matrix().block<3, 4>(0, 0),
        (offset * offset * X_FGs_[i].matrix()).block<3, 4>(0, 0)));
  }
}

// Test various frame property queries.
TEST_F(GeometryStateTest, QueryFrameProperties) {
  const SourceId s_id = SetUpSingleSourceTree();
  const FrameId world = InternalFrame::world_frame_id();

  // Query frame group.
  EXPECT_EQ(geometry_state_.get_frame_group(frames_[0]), 0);
  EXPECT_EQ(geometry_state_.get_frame_group(world),
            InternalFrame::world_frame_group());
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.get_frame_group(FrameId::get_new_id()), std::logic_error,
      "No frame group available for invalid frame id: \\d+");

  // Query frame name.
  EXPECT_EQ(geometry_state_.get_frame_name(frames_[0]), "f0");
  EXPECT_EQ(geometry_state_.get_frame_name(world), "world");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.get_frame_name(FrameId::get_new_id()), std::logic_error,
      "No frame name available for invalid frame id: \\d+");

  // Set the frame poses to query geometry and frame poses.
  FramePoseVector<double> poses;
  for (int i = 0; i < kFrameCount; ++i) poses.set_value(frames_[i], X_PFs_[i]);
  gs_tester_.SetFramePoses(s_id, poses);

  EXPECT_TRUE(
      CompareMatrices(geometry_state_.get_pose_in_world(frames_[0]).matrix(),
                      X_WFs_[0].matrix()));
  EXPECT_TRUE(
      CompareMatrices(geometry_state_.get_pose_in_world(world).matrix(),
                      Isometry3d::Identity().matrix()));
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.get_pose_in_world(FrameId::get_new_id()),
      std::logic_error, "No world pose available for invalid frame id: \\d+");

  // This assumes that geometry parent belongs to frame 0.
  const Isometry3d X_WG = X_WFs_[0] * X_FGs_[0];
  EXPECT_TRUE(CompareMatrices(
      geometry_state_.get_pose_in_world(geometries_[0]).matrix(),
      X_WG.matrix()));
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.get_pose_in_world(GeometryId::get_new_id()),
      std::logic_error,
      "No world pose available for invalid geometry id: \\d+");
  EXPECT_TRUE(CompareMatrices(
      geometry_state_.get_pose_in_world(anchored_geometry_).matrix(),
      X_WA_.matrix()));

  EXPECT_TRUE(
      CompareMatrices(geometry_state_.get_pose_in_parent(frames_[0]).matrix(),
                      X_PFs_[0].matrix()));
  EXPECT_TRUE(
      CompareMatrices(geometry_state_.get_pose_in_parent(world).matrix(),
                      Isometry3d::Identity().matrix()));
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.get_pose_in_parent(FrameId::get_new_id()),
      std::logic_error, "No pose available for invalid frame id: \\d+");
}

TEST_F(GeometryStateTest, TestCollisionCandidates) {
  SetUpSingleSourceTree(Assign::kProximity);

  // The explicit enumeration of candidate pairs. geometries 0 & 1, 2 & 3, and
  // 4 & 5 are siblings, therefore they are not valid candidate pairs. All other
  // combinations _are_.
  set<pair<GeometryId, GeometryId>> expected_candidates =
      {{geometries_[0], geometries_[2]}, {geometries_[0], geometries_[3]},
       {geometries_[0], geometries_[4]}, {geometries_[0], geometries_[5]},
       {geometries_[0], anchored_geometry_},
       {geometries_[1], geometries_[2]}, {geometries_[1], geometries_[3]},
       {geometries_[1], geometries_[4]}, {geometries_[1], geometries_[5]},
       {geometries_[1], anchored_geometry_},
       {geometries_[2], geometries_[4]}, {geometries_[2], geometries_[5]},
       {geometries_[2], anchored_geometry_},
       {geometries_[3], geometries_[4]}, {geometries_[3], geometries_[5]},
       {geometries_[3], anchored_geometry_},
       {geometries_[4], anchored_geometry_},
       {geometries_[5], anchored_geometry_}};

  auto candidates_in_set =
      [](const set<pair<GeometryId, GeometryId>>& candidates,
         const set<pair<GeometryId, GeometryId>>& expected) {
        ::testing::AssertionResult result = ::testing::AssertionSuccess();
        if (candidates != expected) {
          result = ::testing::AssertionFailure();
          auto print_difference = [&result](const auto& set1, const auto& set2,
                                            const char* msg) {
            set<pair<GeometryId, GeometryId>> diff;
            std::set_difference(set1.begin(), set1.end(), set2.begin(),
                                set2.end(), std::inserter(diff, diff.begin()));
            if (!diff.empty()) {
              result << "\n    " << msg;
              for (const auto& p : diff) {
                result << " (" << p.first << ", " << p.second << ")";
              }
            }
          };
          print_difference(
              candidates, expected,
              "The following pairs were reported but not expected:");
          print_difference(expected, candidates,
                           "The following pairs were expected but missing:");
        }
        return result;
      };

  // Confirm initial conditions.
  EXPECT_TRUE(candidates_in_set(geometry_state_.GetCollisionCandidates(),
                                expected_candidates));

  // This assumes that ExcludeCollisionsBetween() (tested below) works.
  while (!expected_candidates.empty()) {
    const auto pair = expected_candidates.begin();
    geometry_state_.ExcludeCollisionsBetween(GeometrySet{pair->first},
                                             GeometrySet{pair->second});
    expected_candidates.erase(pair);
    EXPECT_TRUE(candidates_in_set(geometry_state_.GetCollisionCandidates(),
                                  expected_candidates));
  }
  // We've filtered everything, should report as empty.
  EXPECT_TRUE(candidates_in_set(geometry_state_.GetCollisionCandidates(),
                                expected_candidates));
}

// Test disallowing collisions among members of a group (self collisions).
TEST_F(GeometryStateTest, ExcludeCollisionsWithin) {
  SetUpSingleSourceTree(Assign::kProximity);

  // Pose all of the frames to the specified poses in their parent frame.
  FramePoseVector<double> poses;
  for (int f = 0; f < static_cast<int>(frames_.size()); ++f) {
    poses.set_value(frames_[f], X_PFs_[f]);
  }
  gs_tester_.SetFramePoses(source_id_, poses);
  gs_tester_.FinalizePoseUpdate();

  // This is *non* const; we'll decrement it as we filter more and more
  // collisions.
  int expected_collisions = default_collision_pair_count();

  // Baseline collision - the unfiltered collisions.
  auto pairs = geometry_state_.ComputePointPairPenetration();
  EXPECT_EQ(static_cast<int>(pairs.size()), expected_collisions);

  int next_clique = gs_tester_.peek_next_clique();
  // A GeometrySet with a single frame (and no geometry) should have no change
  // on the outcome.
  geometry_state_.ExcludeCollisionsWithin(
      GeometrySet({frames_[0]}));
  pairs = geometry_state_.ComputePointPairPenetration();
  ASSERT_EQ(static_cast<int>(pairs.size()), expected_collisions);
  // A clique was *not* consumed.
  EXPECT_EQ(gs_tester_.peek_next_clique(), next_clique);

  // A GeometrySet with a single geometry (and no frames) should have no change
  // on the outcome.
  geometry_state_.ExcludeCollisionsWithin(
      GeometrySet({geometries_[0]}));
  pairs = geometry_state_.ComputePointPairPenetration();
  ASSERT_EQ(static_cast<int>(pairs.size()), expected_collisions);
  // A clique was *not* consumed.
  EXPECT_EQ(gs_tester_.peek_next_clique(), next_clique);

  // Frames 0 & 1 do *not* have colliding geometry; adding a filter should have
  // *no* impact on the number of reported collisions.

  // Confirm that geometry pairs (i, j) are reported as initially _not_ filtered
  // from collisions where i is a geometry belonging to frame 0 and j is a
  // geometry belonging to frame 1.
  for (int i = 0; i < kGeometryCount; ++i) {
    for (int j = kGeometryCount; j < kGeometryCount * 2; ++j) {
      EXPECT_FALSE(geometry_state_.CollisionFiltered(
          geometries_[i], geometries_[j]));
    }
  }
  geometry_state_.ExcludeCollisionsWithin(
      GeometrySet({anchored_geometry_}, {frames_[0], frames_[1]}));
  pairs = geometry_state_.ComputePointPairPenetration();
  ASSERT_EQ(static_cast<int>(pairs.size()), expected_collisions);
  // Now confirm that collision filtering is reported.
  for (int i = 0; i < kGeometryCount; ++i) {
    for (int j = kGeometryCount; j < kGeometryCount * 2; ++j) {
      EXPECT_TRUE(geometry_state_.CollisionFiltered(
          geometries_[i], geometries_[j]));
    }
  }

  // Frame 2 has *two* geometries that collide with the anchored geometry. This
  // eliminates those collisions.
  geometry_state_.ExcludeCollisionsWithin(
      GeometrySet({anchored_geometry_}, {frames_[2]}));
  expected_collisions -= 2;
  pairs = geometry_state_.ComputePointPairPenetration();
  ASSERT_EQ(static_cast<int>(pairs.size()), expected_collisions);
}

// Test disallowing collision between members fo two groups.
TEST_F(GeometryStateTest, ExcludeCollisionsBetween) {
  SetUpSingleSourceTree(Assign::kProximity);

  // Pose all of the frames to the specified poses in their parent frame.
  FramePoseVector<double> poses;
  for (int f = 0; f < static_cast<int>(frames_.size()); ++f) {
    poses.set_value(frames_[f], X_PFs_[f]);
  }
  gs_tester_.SetFramePoses(source_id_, poses);
  gs_tester_.FinalizePoseUpdate();

  // This is *non* const; we'll decrement it as we filter more and more
  // collisions.
  int expected_collisions = default_collision_pair_count();

  // Baseline collision - the unfiltered collisions.
  auto pairs = geometry_state_.ComputePointPairPenetration();
  EXPECT_EQ(static_cast<int>(pairs.size()), expected_collisions);

  // Frames 0 & 1 do *not* have colliding geometry; adding a filter should have
  // *no* impact on the number of reported collisions.
  geometry_state_.ExcludeCollisionsBetween(
      GeometrySet{frames_[0], frames_[1]},
      GeometrySet(anchored_geometry_));
  pairs = geometry_state_.ComputePointPairPenetration();
  ASSERT_EQ(static_cast<int>(pairs.size()), expected_collisions);

  // Frame 2 has *two* geometries that collide with the anchored geometry. Test
  // that the removal of collision between frame 2's geometries and the anchored
  // geometry leave the collisions *between* geometries g4 and g5 intact.
  geometry_state_.ExcludeCollisionsBetween(GeometrySet{frames_[2]},
                                           GeometrySet{anchored_geometry_});
  expected_collisions -= 2;
  pairs = geometry_state_.ComputePointPairPenetration();
  ASSERT_EQ(static_cast<int>(pairs.size()), expected_collisions);
}

// Test collision filtering configuration when the input GeometrySet includes
// geometries that *do not* have a proximity role.
TEST_F(GeometryStateTest, NonProximityRoleInCollisionFilter) {
  SetUpSingleSourceTree(Assign::kProximity);

  // Pose all of the frames to the specified poses in their parent frame.
  FramePoseVector<double> poses;
  for (int f = 0; f < static_cast<int>(frames_.size()); ++f) {
    poses.set_value(frames_[f], X_PFs_[f]);
  }
  gs_tester_.SetFramePoses(source_id_, poses);
  gs_tester_.FinalizePoseUpdate();

  // This is *non* const; we'll decrement it as we filter more and more
  // collisions.
  int expected_collisions = default_collision_pair_count();

  // Baseline collision - the unfiltered collisions.
  auto pairs = geometry_state_.ComputePointPairPenetration();
  EXPECT_EQ(static_cast<int>(pairs.size()), expected_collisions);

  // Add a new collision element to the third frame. Position it so that the
  // sphere penetrates with both the previous geometries on the frame and
  // the anchored geometry. Initially, assign no proximity role to the new
  // geometry.
  //   - No proximity role implies no additional contacts are reported.
  //   - However, when a role is assigned to the new geometry, it will *only*
  //     report one new collision (with the anchored geometry). Collisions
  //     between the new geometry and the previous geometries should be
  //     automatically filtered because they are rigidly affixed to the same
  //     frame.

  // Documentation on the single source tree indicates that the previous spheres
  // are at (5, 0, 0) and (6, 0, 0), respectively. Split the distance to put the
  // new geometry in a penetrating configuration.
  const Isometry3d pose{Translation3d{5.5, 0, 0}};
  const string name("added_sphere");
  GeometryId added_id = geometry_state_.RegisterGeometry(
      source_id_, frames_[2],
      make_unique<GeometryInstance>(pose, make_unique<Sphere>(1), name));
  gs_tester_.FinalizePoseUpdate();

  // No change in number of collisions.
  pairs = geometry_state_.ComputePointPairPenetration();
  EXPECT_EQ(static_cast<int>(pairs.size()), expected_collisions);

  // Attempting to filter collisions between a geometry with no proximity role
  // and other geometry should have no effect on the number of collisions.
  geometry_state_.ExcludeCollisionsBetween(GeometrySet{added_id},
                                           GeometrySet{anchored_geometry_});
  pairs = geometry_state_.ComputePointPairPenetration();
  EXPECT_EQ(static_cast<int>(pairs.size()), expected_collisions);

  // If we assign a role, the collisions go up by one. The previous attempt
  // to filter collisions was a no op because added_id didn't have a proximity
  // role. So, collisions between added_id and anchored_geometry_ have not been
  // filtered.
  geometry_state_.AssignRole(source_id_, added_id, ProximityProperties());
  pairs = geometry_state_.ComputePointPairPenetration();
  EXPECT_EQ(static_cast<int>(pairs.size()), expected_collisions + 1);

  // Now if we filter it, it should get removed.
  geometry_state_.ExcludeCollisionsBetween(GeometrySet{added_id},
                                           GeometrySet{anchored_geometry_});
  pairs = geometry_state_.ComputePointPairPenetration();
  EXPECT_EQ(static_cast<int>(pairs.size()), expected_collisions);
}

// Tests the documented error conditions of ExcludeCollisionsWithin.
TEST_F(GeometryStateTest, SelfCollisionFilterExceptions) {
  SetUpSingleSourceTree();

  // NOTE: a collision group with a single frame or geometry doesn't exercise
  // self-collision filtering logic.
  const GeometrySet set_bad_frame{FrameId::get_new_id(), FrameId::get_new_id()};
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.ExcludeCollisionsWithin(set_bad_frame), std::logic_error,
      "Referenced frame \\d+ has not been registered.");

  const GeometrySet set_bad_geometry{GeometryId::get_new_id(),
                                     GeometryId::get_new_id()};
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.ExcludeCollisionsWithin(set_bad_geometry),
      std::logic_error,
      "Geometry set includes a geometry id that doesn't belong to the "
          "SceneGraph: \\d+");
}

// Tests the documented error conditions of ExcludeCollisionsWithin.
TEST_F(GeometryStateTest, CrossCollisionFilterExceptions) {
  SetUpSingleSourceTree();

  const GeometrySet set_bad_frame{FrameId::get_new_id()};
  const GeometrySet set_good_frame{frames_[0]};
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.ExcludeCollisionsBetween(set_bad_frame,
                                               set_good_frame),
      std::logic_error,
      "Referenced frame \\d+ has not been registered.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.ExcludeCollisionsBetween(set_good_frame,
                                               set_bad_frame),
      std::logic_error,
      "Referenced frame \\d+ has not been registered.");

  const GeometrySet set_bad_geometry{GeometryId::get_new_id()};
  const GeometrySet set_good_geometry{geometries_[0]};
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.ExcludeCollisionsBetween(set_bad_geometry,
                                               set_good_geometry),
      std::logic_error,
      "Geometry set includes a geometry id that doesn't belong to the "
          "SceneGraph: \\d+");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.ExcludeCollisionsBetween(set_good_geometry,
                                               set_bad_geometry),
      std::logic_error,
      "Geometry set includes a geometry id that doesn't belong to the "
          "SceneGraph: \\d+");
}

// Test that the appropriate error messages are dispatched.
TEST_F(GeometryStateTest, CollisionFilteredExceptions) {
  // Initialize tree *without* any proximity roles assigned.
  SetUpSingleSourceTree();

  // Assign proximity to a *single* geometry (which won't cause any automatic
  // filtering.
  geometry_state_.AssignRole(source_id_, geometries_[0], ProximityProperties());

  // Case: First argument does not have a proximity role.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.CollisionFiltered(geometries_[1], geometries_[0]),
      std::logic_error,
      ".* " + to_string(geometries_[1]) + " does not have a proximity role");

  // Case: Second argument does not have a proximity role.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.CollisionFiltered(geometries_[0], geometries_[1]),
      std::logic_error,
      ".* " + to_string(geometries_[1]) + " does not have a proximity role");

  // Case: Neither argument has a proximity role.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.CollisionFiltered(geometries_[1], geometries_[2]),
      std::logic_error,
      ".* neither id has a proximity role");

  // Assign proximity to the *other* geometry on frame 1 -- triggering collision
  // filtering between them.
  geometry_state_.AssignRole(source_id_, geometries_[1], ProximityProperties());

  // Base case: Two geometries on same frame *are* filtered.
  EXPECT_TRUE(
      geometry_state_.CollisionFiltered(geometries_[0], geometries_[1]));

  const GeometryId bad_id = GeometryId::get_new_id();

  // Case: First argument is bad.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.CollisionFiltered(bad_id, geometries_[0]),
      std::logic_error,
      "Can't report collision filter status between geometries .* " +
          to_string(bad_id) + " is not a valid geometry");

  // Case: Second argument is bad.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.CollisionFiltered(geometries_[0], bad_id),
      std::logic_error,
      "Can't report collision filter status between geometries .* " +
          to_string(bad_id) + " is not a valid geometry");

  // Case: Both arguments are bad.
  DRAKE_EXPECT_THROWS_MESSAGE(geometry_state_.CollisionFiltered(bad_id, bad_id),
                              std::logic_error,
                              "Can't report collision filter status between "
                              "geometries .* neither id is a valid geometry");
}

// Tests the ability to query for a geometry from the name of a geometry.
TEST_F(GeometryStateTest, GetGeometryIdFromName) {
  SetUpSingleSourceTree(Assign::kProximity);

  // Frame i has geometries f * kFrameCount + g, where g ∈ [0, kGeometryCount).
  for (int f = 0; f < kFrameCount; ++f) {
    for (int g = 0; g < kGeometryCount; ++g) {
      int g_index = f * kGeometryCount + g;
      const GeometryId expected_id = geometries_[g_index];
      // Look up with the canonical name.
      EXPECT_EQ(geometry_state_.GetGeometryFromName(frames_[f],
                                                    Role::kProximity,
                                                    geometry_names_[g_index]),
                expected_id);
      // Look up with non-canonical name.
      EXPECT_EQ(geometry_state_.GetGeometryFromName(
                    frames_[f], Role::kProximity,
                    " " + geometry_names_[g_index]),
                expected_id);
    }
  }

  // Grab anchored.
  EXPECT_EQ(
      geometry_state_.GetGeometryFromName(InternalFrame::world_frame_id(),
                                          Role::kProximity, anchored_name_),
      anchored_geometry_);

  // Failure cases.

  // Bad frame id.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.GetGeometryFromName(FrameId::get_new_id(),
                                          Role::kUnassigned, "irrelevant"),
      std::logic_error, "Referenced frame \\d+ has not been registered.");

  // Bad *anchored* geometry name.
  const FrameId world_id = gs_tester_.get_world_frame();
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.GetGeometryFromName(world_id, Role::kUnassigned, "bad"),
      std::logic_error,
      "The frame 'world' .\\d+. has no geometry with the role 'unassigned' "
      "and the canonical name '.+'");

  // Bad *dynamic* geometry name.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.GetGeometryFromName(frames_[0], Role::kUnassigned,
                                          "bad_name"),
      std::logic_error,
      "The frame '.+?' .\\d+. has no geometry with the role 'unassigned' and "
      "the canonical name '.+'");

  // Multiple unassigned geometries with the same name.

  const string dummy_name("duplicate");
  for (int i = 0; i < 2; ++i) {
    geometry_state_.RegisterGeometry(
        source_id_, frames_[0],
        make_unique<GeometryInstance>(Isometry3d::Identity(),
                                      make_unique<Sphere>(1), dummy_name));
  }
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.GetGeometryFromName(frames_[0], Role::kUnassigned,
                                          dummy_name),
      std::logic_error,
      "The frame '.+?' .\\d+. has multiple geometries with the role "
      "'unassigned' and the canonical name '.+'");
}

// Confirms that the name *stored* with the geometry is the trimmed version of
// the requested name.
TEST_F(GeometryStateTest, GeometryNameStorage) {
  SetUpSingleSourceTree();

  const string name = "unique test name";

  // White space trimmed off and new string stored.
  {
    const GeometryId id = geometry_state_.RegisterGeometry(
        source_id_, frames_[0],
        make_unique<GeometryInstance>(
            Isometry3d::Identity(), make_unique<Sphere>(1), " " + name));
    EXPECT_EQ(geometry_state_.GetName(id), name);
  }

  // Valid name that is unchanged after trimming is stored as is.
  // Note: This assigns a geometry fo the *same* name to a *different* frame.
  {
    const GeometryId id = geometry_state_.RegisterGeometry(
        source_id_, frames_[1],
        make_unique<GeometryInstance>(
            Isometry3d::Identity(), make_unique<Sphere>(1), name));
    EXPECT_EQ(geometry_state_.GetName(id), name);
  }
}

// Tests the logic for confirming if a name is valid or not.
TEST_F(GeometryStateTest, GeometryNameValidation) {
  SetUpSingleSourceTree(Assign::kProximity);

  // Case: Invalid frame should throw (regardless of name contents or role).
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.IsValidGeometryName(FrameId::get_new_id(),
                                          Role::kProximity, ""),
      std::logic_error, "Given frame id is not valid: \\d+");

  auto expect_bad_name = [this](const string& name,
                                const string& exception_message,
                                const string& printable_name) {
    EXPECT_FALSE(geometry_state_.IsValidGeometryName(frames_[0],
                                                     Role::kProximity, name))
        << "Failed on input name: " << printable_name;
  };

  // Unique in world frame.
  const FrameId world = InternalFrame::world_frame_id();
  EXPECT_FALSE(geometry_state_.IsValidGeometryName(world, Role::kProximity,
                                                   anchored_name_));
  EXPECT_TRUE(geometry_state_.IsValidGeometryName(world, Role::kIllustration,
                                                  anchored_name_));
  EXPECT_TRUE(geometry_state_.IsValidGeometryName(world, Role::kProximity,
                                                  anchored_name_ + "2"));

  // Invalid cases:
  // Empty.
  expect_bad_name("", "The proposed geometry name is empty", "");

  // Nothing but whitespace.
  const string whitespace_message{
      "The proposed geometry name contains only whitespace"};
  expect_bad_name(" ", whitespace_message, "' '");
  expect_bad_name("\t", whitespace_message, "'\\t'");
  expect_bad_name(" \t", whitespace_message, "' \\t'");

  // Case: Valid (as a control case).
  const string unique = "unique";
  EXPECT_TRUE(geometry_state_.IsValidGeometryName(frames_[0], Role::kProximity,
                                                  unique));

  // Querying with non-canonical names test as the canonical name.
  const vector<string> names{" " + unique, unique + " ", " " + unique + " "};
  for (const auto& name : names) {
    EXPECT_TRUE(geometry_state_.IsValidGeometryName(frames_[0],
                                                    Role::kProximity, name));
  }

  // Test potential duplicates.

  // A name with the same role will fail.
  EXPECT_FALSE(geometry_state_.IsValidGeometryName(
      frames_[0], Role::kProximity,
      gs_tester_.get_geometries().at(geometries_[0]).name()));

  // A name with the *different* role will pass.
  EXPECT_TRUE(geometry_state_.IsValidGeometryName(
      frames_[0], Role::kIllustration,
      gs_tester_.get_geometries().at(geometries_[0]).name()));

  // Case: Whitespace that SDF nevertheless considers not whitespace.
  // Update this when the following sdformat issue is resolved:
  // https://bitbucket.org/osrf/sdformat/issues/194/string-trimming-only-considers-space-and
  for (const string& s : {"\n", " \n\t", " \f", "\v", "\r", "\ntest"}) {
    EXPECT_TRUE(geometry_state_.IsValidGeometryName(frames_[0],
                                                    Role::kProximity, s));
  }
}

// Simply tests that when a role is assigned to a geometry, that it successfully
// reports that it has that role.
TEST_F(GeometryStateTest, AssignRolesToGeometry) {
  SetUpSingleSourceTree();

  // We need at least 8 geometries to run through all role permutations. Add
  // geometries until we're there.
  const Isometry3d pose = Isometry3d::Identity();
  for (int i = 0; i < 8 - single_tree_dynamic_geometry_count(); ++i) {
    const string name = "new_geom" + std::to_string(i);
    geometries_.push_back(geometry_state_.RegisterGeometry(
        source_id_, frames_[0],
        make_unique<GeometryInstance>(pose, make_unique<Sphere>(1), name)));
  }

  auto set_roles = [this](GeometryId id, bool set_proximity,
                          bool set_perception, bool set_illustration) {
    if (set_proximity) {
      geometry_state_.AssignRole(source_id_, id, ProximityProperties());
    }
    PerceptionProperties p = render_engine_->accepting_properties();
    p.AddProperty("label", "id", RenderLabel(10));
    if (set_perception) {
      geometry_state_.AssignRole(source_id_, id, p);
    }
    if (set_illustration) {
      geometry_state_.AssignRole(source_id_, id, IllustrationProperties());
    }
  };

  auto has_expected_roles =
      [this](GeometryId id, bool has_proximity, bool has_perception,
             bool has_illustration) -> ::testing::AssertionResult {
    const InternalGeometry* geometry = gs_tester_.GetGeometry(id);
    bool passes = true;
    ::testing::AssertionResult failure = ::testing::AssertionFailure();
    if (has_proximity != (geometry->proximity_properties() != nullptr)) {
      failure << "Proximity role: "
              << (has_proximity ? "expected, but not found"
                                : "not expected, but found. ");
      passes = false;
    }
    if (has_perception != (geometry->perception_properties() != nullptr)) {
      failure << "Perception role: "
              << (has_perception ? "expected, but not found"
                                 : "not expected, but found. ");
      passes = false;
    }
    if (has_perception !=
        (geometry->render_index(kDummyRenderName).has_value())) {
      failure << "Perception role: "
              << (has_perception ? "expected, but no render index found"
                                 : "not expected, but render index found");
      passes = false;
    }
    if (has_illustration != (geometry->illustration_properties() != nullptr)) {
      failure << "Illustration role: "
              << (has_illustration ? "expected, but not found"
                                   : "not expected, but found. ");
      passes = false;
    }
    if (passes)
      return ::testing::AssertionSuccess();
    else
      return failure;
  };

  // Given three role types, assign all eight types of assignments.
  for (int i = 0; i < 8; ++i) {
    const bool proximity = i & 0x1;
    const bool illustration = i & 0x2;
    const bool perception = i & 0x4;
    const GeometryId id = geometries_[i];
    EXPECT_TRUE(has_expected_roles(id, false, false, false))
              << "Geometry " << id << " at index (" << i
              << ") didn't start without roles";
    set_roles(id, proximity, perception, illustration);
    EXPECT_TRUE(has_expected_roles(id, proximity, perception, illustration))
              << "Incorrect roles for geometry " << id << " at index (" << i
              << ").";
  }

  // Confirm it works on anchored geometry. Pick, arbitrarily, assigning
  // proximity and illustration roles.
  EXPECT_TRUE(has_expected_roles(anchored_geometry_, false, false, false));
  set_roles(anchored_geometry_, true, false, true);
  EXPECT_TRUE(has_expected_roles(anchored_geometry_, true, false, true));
}

// Tests the various Get*Properties(GeometryId) methods.
TEST_F(GeometryStateTest, RoleLookUp) {
  SetUpSingleSourceTree(Assign::kProximity | Assign::kIllustration |
                        Assign::kPerception);
  GeometryId no_role_id = geometry_state_.RegisterGeometry(
      source_id_, frames_[0],
      make_unique<GeometryInstance>(Isometry3d::Identity(),
                                    make_unique<Sphere>(0.5), "no_roles"));
  GeometryId invalid_id = GeometryId::get_new_id();

  // Invalid id throws.
  EXPECT_THROW(geometry_state_.GetProximityProperties(invalid_id),
               std::logic_error);
  EXPECT_THROW(geometry_state_.GetIllustrationProperties(invalid_id),
               std::logic_error);
  EXPECT_THROW(geometry_state_.GetPerceptionProperties(invalid_id),
               std::logic_error);

  // Missing role returns nullptr.
  EXPECT_EQ(geometry_state_.GetProximityProperties(no_role_id), nullptr);
  EXPECT_EQ(geometry_state_.GetIllustrationProperties(no_role_id), nullptr);
  EXPECT_EQ(geometry_state_.GetPerceptionProperties(no_role_id), nullptr);

  // Non-null when properties exist. Actual values not tested here; they've
  // been tested elsewhere.
  EXPECT_NE(geometry_state_.GetProximityProperties(geometries_[0]), nullptr);
  EXPECT_NE(geometry_state_.GetIllustrationProperties(geometries_[0]), nullptr);
  EXPECT_NE(geometry_state_.GetPerceptionProperties(geometries_[0]), nullptr);
}

// Tests that properties assigned to a geometry instance lead to the resulting
// geometry having the appropriate roles assigned.
TEST_F(GeometryStateTest, InstanceRoleAssignment) {
  const SourceId s_id = NewSource();
  const FrameId f_id =
      geometry_state_.RegisterFrame(s_id, GeometryFrame("frame"));

  auto make_instance = [this](const string& name) {
    return make_unique<GeometryInstance>(
        instance_pose_, make_unique<Sphere>(1.0), name);
  };

  PerceptionProperties perception_props =
      render_engine_->accepting_properties();
  perception_props.AddProperty("label", "id", RenderLabel(10));

  // Case: no properties assigned leaves geometry with no roles.
  {
    const GeometryId g_id = geometry_state_.RegisterGeometry(
        s_id, f_id, make_instance("instance1"));
    const InternalGeometry* geometry = gs_tester_.GetGeometry(g_id);
    EXPECT_FALSE(geometry->has_proximity_role());
    EXPECT_FALSE(geometry->has_illustration_role());
    EXPECT_FALSE(geometry->has_perception_role());
  }

  // Case: Only proximity properties provided.
  {
    auto instance = make_instance("instance2");
    instance->set_proximity_properties(ProximityProperties());
    const GeometryId g_id =
        geometry_state_.RegisterGeometry(s_id, f_id, move(instance));

    const InternalGeometry* geometry = gs_tester_.GetGeometry(g_id);
    EXPECT_TRUE(geometry->has_proximity_role());
    EXPECT_FALSE(geometry->has_illustration_role());
    EXPECT_FALSE(geometry->has_perception_role());
  }

  // Case: Only illustration properties provided.
  {
    auto instance = make_instance("instance3");
    instance->set_illustration_properties(IllustrationProperties());
    const GeometryId g_id =
        geometry_state_.RegisterGeometry(s_id, f_id, move(instance));

    const InternalGeometry* geometry = gs_tester_.GetGeometry(g_id);
    EXPECT_FALSE(geometry->has_proximity_role());
    EXPECT_TRUE(geometry->has_illustration_role());
    EXPECT_FALSE(geometry->has_perception_role());
  }

  // Case: Only perception properties provided.
  {
    auto instance = make_instance("instance4");
    instance->set_perception_properties(perception_props);
    const GeometryId g_id =
        geometry_state_.RegisterGeometry(s_id, f_id, move(instance));

    const InternalGeometry* geometry = gs_tester_.GetGeometry(g_id);
    EXPECT_FALSE(geometry->has_proximity_role());
    EXPECT_FALSE(geometry->has_illustration_role());
    EXPECT_TRUE(geometry->has_perception_role());
  }

  // Case: All properties provided.
  {
    auto instance = make_instance("instance5");
    instance->set_proximity_properties(ProximityProperties());
    instance->set_illustration_properties(IllustrationProperties());
    instance->set_perception_properties(perception_props);
    const GeometryId g_id =
        geometry_state_.RegisterGeometry(s_id, f_id, move(instance));

    const InternalGeometry* geometry = gs_tester_.GetGeometry(g_id);
    EXPECT_TRUE(geometry->has_proximity_role());
    EXPECT_TRUE(geometry->has_illustration_role());
    EXPECT_TRUE(geometry->has_perception_role());
  }
}

// Tests that the property values created get correctly propagated to the
// target geometry.
TEST_F(GeometryStateTest, RolePropertyValueAssignment) {
  SetUpSingleSourceTree();
  // Tests for proximity properties and assumes the same holds true for the
  // other role property types.

  ProximityProperties source;
  const string& default_group = source.default_group_name();
  source.AddProperty(default_group, "prop1", 7);
  source.AddProperty(default_group, "prop2", 10);
  const string group1("group1");
  source.AddProperty(group1, "propA", 7.5);
  source.AddProperty(group1, "propB", "test");

  geometry_state_.AssignRole(source_id_, geometries_[0], source);
  const ProximityProperties* read =
      gs_tester_.GetGeometry(geometries_[0])->proximity_properties();
  ASSERT_NE(read, nullptr);

  // Test groups.
  ASSERT_EQ(source.num_groups(), read->num_groups());
  ASSERT_TRUE(read->HasGroup(group1));
  ASSERT_TRUE(read->HasGroup(default_group));

  // Utility for counting properties in a group.
  auto num_group_properties = [](const ProximityProperties& properties,
                                 const string& group_name) {
    const auto& group = properties.GetPropertiesInGroup(group_name);
    return static_cast<int>(group.size());
  };

  // Utility for counting properties in a full set of properties.
  auto num_total_properties =
      [num_group_properties](const ProximityProperties& properties) {
        int count = 0;
        for (const auto& group_name : properties.GetGroupNames()) {
          count += num_group_properties(properties, group_name);
        }
        return count;
      };

  // Test properties.
  EXPECT_EQ(num_total_properties(source), num_total_properties(*read));

  EXPECT_EQ(num_group_properties(source, default_group),
            num_group_properties(*read, default_group));
  EXPECT_EQ(source.GetProperty<int>(default_group, "prop1"),
            read->GetProperty<int>(default_group, "prop1"));
  EXPECT_EQ(source.GetProperty<int>(default_group, "prop2"),
            read->GetProperty<int>(default_group, "prop2"));

  EXPECT_EQ(num_group_properties(source, group1),
            num_group_properties(*read, group1));
  EXPECT_EQ(source.GetProperty<double>(group1, "propA"),
            read->GetProperty<double>(group1, "propA"));
  EXPECT_EQ(source.GetProperty<string>(group1, "propB"),
            read->GetProperty<string>(group1, "propB"));
}

// Tests the conditions in which `AssignRole()` throws an exception.
TEST_F(GeometryStateTest, RoleAssignExceptions) {
  SetUpSingleSourceTree();

  PerceptionProperties perception_props =
      render_engine_->accepting_properties();
  perception_props.AddProperty("label", "id", RenderLabel(10));

  // NOTE: On the basis that all AssignRole variants ultimately call the same
  // underlying method, this only exercises one variant to represent all. If
  // they no longer invoke the same underlying method, this test should change
  // accordingly.

  // Invalid source.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.AssignRole(SourceId::get_new_id(), geometries_[0],
                                 ProximityProperties()),
      std::logic_error,
      "Referenced geometry source \\d+ is not registered.");

  // Invalid geometry id.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.AssignRole(source_id_, GeometryId::get_new_id(),
                                 ProximityProperties()),
      std::logic_error,
      "Referenced geometry \\d+ has not been registered.");

  // Geometry not owned by source.
  const SourceId other_source = geometry_state_.RegisterNewSource("alt_source");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.AssignRole(other_source, geometries_[0],
                                 ProximityProperties()),
      std::logic_error,
      "Given geometry id \\d+ does not belong to the given source .*");

  // Redefinition of role - test each role individually to make sure it has
  // the right error message.
  EXPECT_NO_THROW(geometry_state_.AssignRole(source_id_, geometries_[0],
                                             ProximityProperties()));
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.AssignRole(source_id_, geometries_[0],
                                 ProximityProperties()),
      std::logic_error,
      "Geometry already has proximity role assigned");

  EXPECT_NO_THROW(
      geometry_state_.AssignRole(source_id_, geometries_[0], perception_props));
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.AssignRole(source_id_, geometries_[0], perception_props),
      std::logic_error, "Geometry already has perception role assigned");

  EXPECT_NO_THROW(geometry_state_.AssignRole(source_id_, geometries_[0],
                                             IllustrationProperties()));
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.AssignRole(source_id_, geometries_[0],
                                 IllustrationProperties()),
      std::logic_error,
      "Geometry already has illustration role assigned");

  // Addition of geometry with duplicate name -- no problem. Assigning it a
  // duplicate role -- bad.
  const GeometryId new_id = geometry_state_.RegisterGeometry(
      source_id_, frames_[0],
      make_unique<GeometryInstance>(
          Isometry3d::Identity(), make_unique<Sphere>(1), geometry_names_[0]));
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.AssignRole(source_id_, new_id, ProximityProperties()),
      std::logic_error,
      "The name .* has already been used by a geometry with the 'proximity' "
      "role.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.AssignRole(source_id_, new_id, IllustrationProperties()),
      std::logic_error,
      "The name .* has already been used by a geometry with the 'illustration' "
      "role.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.AssignRole(source_id_, new_id, perception_props),
      std::logic_error,
      "The name .* has already been used by a geometry with the 'perception' "
      "role.");
}

// Tests the functionality that counts the number of children geometry a frame
// has for each role.
TEST_F(GeometryStateTest, ChildGeometryRoleCount) {
  // Defaults to *no* roles being set.
  SetUpSingleSourceTree();

  auto expected_roles = [this](
      FrameId f_id, int num_proximity, int num_perception,
      int num_illustration) -> ::testing::AssertionResult {
    bool success = true;
    ::testing::AssertionResult failure = ::testing::AssertionFailure();
    vector<pair<Role, int>> roles{{Role::kProximity, num_proximity},
                                  {Role::kPerception, num_perception},
                                  {Role::kIllustration, num_illustration}};
    for (const auto& pair : roles) {
      const Role role = pair.first;
      const int expected_count = pair.second;
      const int actual_count =
          geometry_state_.NumGeometriesWithRole(f_id, role);
      if (actual_count != expected_count) {
        success = false;
        failure << "\nExpected " << expected_count << " geometries with the "
                << role << " role. Found " << actual_count;
      }
    }
    if (success)
      return ::testing::AssertionSuccess();
    else
      return failure;
  };

  // Assert initial conditions.
  int proximity_count = 0;
  int perception_count = 0;
  int illustration_count = 0;
  const FrameId f_id = frames_[0];
  ASSERT_TRUE(expected_roles(f_id, proximity_count, perception_count,
                             illustration_count));

  PerceptionProperties perception_props =
      render_engine_->accepting_properties();
  perception_props.AddProperty("label", "id", RenderLabel(10));

  // Confirm the two geometries I'm going to play with belong to the same frame.
  const GeometryId g_id1 = geometries_[0];
  const GeometryId g_id2 = geometries_[1];
  ASSERT_EQ(f_id, geometry_state_.GetFrameId(g_id1));
  ASSERT_EQ(f_id, geometry_state_.GetFrameId(g_id2));

  // Now start assigning roles and confirm results. Do *not* re-order these
  // tests; the expected results accumulate.
  geometry_state_.AssignRole(source_id_, g_id1, ProximityProperties());
  ++proximity_count;
  ASSERT_TRUE(expected_roles(f_id, proximity_count, perception_count,
                             illustration_count));

  geometry_state_.AssignRole(source_id_, g_id2, IllustrationProperties());
  ++illustration_count;
  ASSERT_TRUE(expected_roles(f_id, proximity_count, perception_count,
                             illustration_count));

  geometry_state_.AssignRole(source_id_, g_id1, IllustrationProperties());
  ++illustration_count;
  ASSERT_TRUE(expected_roles(f_id, proximity_count, perception_count,
                             illustration_count));

  geometry_state_.AssignRole(source_id_, g_id1, perception_props);
  ++perception_count;
  ASSERT_TRUE(expected_roles(f_id, proximity_count, perception_count,
                             illustration_count));

  geometry_state_.AssignRole(source_id_, g_id2, perception_props);
  ++perception_count;
  ASSERT_TRUE(expected_roles(f_id, proximity_count, perception_count,
                             illustration_count));

  geometry_state_.AssignRole(source_id_, g_id2, ProximityProperties());
  ++proximity_count;
  ASSERT_TRUE(expected_roles(f_id, proximity_count, perception_count,
                             illustration_count));

  // Now test against anchored geometry by passing in the world frame.
  const FrameId world_id = InternalFrame::world_frame_id();
  ASSERT_TRUE(expected_roles(world_id, 0, 0, 0));
  geometry_state_.AssignRole(source_id_, anchored_geometry_,
                             ProximityProperties());
  ASSERT_TRUE(expected_roles(world_id, 1, 0, 0));
  geometry_state_.AssignRole(source_id_, anchored_geometry_, perception_props);
  ASSERT_TRUE(expected_roles(world_id, 1, 1, 0));
  geometry_state_.AssignRole(source_id_, anchored_geometry_,
                             IllustrationProperties());
  ASSERT_TRUE(expected_roles(world_id, 1, 1, 1));
}

// Confirms that assigning a proximity role to a mesh is a no-op. If it
// *weren't* no-op, the ProximityEngine would abort; so not aborting is
// correlated with its no-op-ness. This test will go away when meshes are fully
// supported in collision.
TEST_F(GeometryStateTest, ProximityRoleOnMesh) {
  SetUpSingleSourceTree();

  // Add a mesh to a frame.
  const GeometryId mesh_id = geometry_state_.RegisterGeometry(
      source_id_, frames_[0],
      make_unique<GeometryInstance>(Isometry3d::Identity(),
                                    make_unique<Mesh>("path", 1.0), "mesh"));
  const InternalGeometry* mesh = gs_tester_.GetGeometry(mesh_id);
  ASSERT_NE(mesh, nullptr);
  ASSERT_FALSE(mesh->has_proximity_role());
  geometry_state_.AssignRole(source_id_, mesh_id, ProximityProperties());
  ASSERT_FALSE(mesh->has_proximity_role());
}

// Confirms that attempting to remove the "unassigned" role has no effect.
TEST_F(GeometryStateTest, RemoveUnassignedRole) {
  SetUpSingleSourceTree(Assign::kProximity | Assign::kIllustration |
      Assign::kPerception);

  EXPECT_EQ(
      geometry_state_.RemoveRole(source_id_, geometries_[0], Role::kUnassigned),
      0);
  EXPECT_EQ(
      geometry_state_.RemoveRole(source_id_, frames_[0], Role::kUnassigned),
      0);

  // Confirm that all geometries still have all roles.
  for (GeometryId id : geometries_) {
    const InternalGeometry* geometry = gs_tester_.GetGeometry(id);
    EXPECT_TRUE(geometry->has_proximity_role());
    EXPECT_TRUE(geometry->has_illustration_role());
    EXPECT_TRUE(geometry->has_perception_role());
  }
}

// Test the removal of a geometry from a particular renderer. This implicitly
// checks the logic in RemoveFromRendererUnchecked (exercised by both the
// Geometry and Frame variants of the RemoveFromRenderer() methods).
TEST_F(GeometryStateTest, RemoveGeometryFromRenderer) {
  // Add an additional renderer *before* populating the world (as required).
  const string other_renderer_name = "alt_renderer";
  DummyRenderEngine* other_renderer{nullptr};
  {
    auto new_renderer = make_unique<DummyRenderEngine>();
    other_renderer = new_renderer.get();
    geometry_state_.AddRenderer(other_renderer_name, move(new_renderer));
  }
  SetUpSingleSourceTree(Assign::kPerception);

  // Each geometry must have a render index for the default render engine. In
  // addition,
  //   a) have an index for the "other" render engine, xor
  //   b) be present in the `removed_from_renderer` set.
  auto confirm_renderers = [=](set<GeometryId> removed_from_renderer) {
    set<GeometryId> ids(geometries_.begin(), geometries_.end());
    ids.insert(anchored_geometry_);
    for (GeometryId id : ids) {
      // All should have render indices in the dummy renderer.
      EXPECT_TRUE(gs_tester_.GetGeometry(id)
                      ->render_index(kDummyRenderName)
                      .has_value());
      // Should have a render index if it is *not* in the removed set.
      EXPECT_EQ(gs_tester_.GetGeometry(id)
                    ->render_index(other_renderer_name)
                    .has_value(),
                removed_from_renderer.count(id) == 0);
    }
  };
  set<GeometryId> removed_ids;

  // Confirm geometries assigned to _both_ renderers.
  confirm_renderers(removed_ids);

  // Configure the renderer to return the _last_ geometry (anchored) to be
  // swapped with the removed geometry.
  const RenderIndex anchored_old_index =
      *gs_tester_.GetGeometry(anchored_geometry_)
          ->render_index(other_renderer_name);
  other_renderer->set_moved_index(anchored_old_index);

  // Case: Remove geometry from a single renderer (should have one left).
  const GeometryId remove_id = geometries_[0];
  const RenderIndex old_index_for_zero =
      *gs_tester_.GetGeometry(remove_id)->render_index(other_renderer_name);
  EXPECT_EQ(geometry_state_.RemoveFromRenderer(other_renderer_name, source_id_,
                                               remove_id),
            1);
  removed_ids.insert(remove_id);
  confirm_renderers(removed_ids);

  // Confirm that the anchored geometry's render index has moved to the slot
  // freed up by the removed geometry.
  EXPECT_EQ(*gs_tester_.GetGeometry(anchored_geometry_)
      ->render_index(other_renderer_name),
            old_index_for_zero);

  // Case: Try removing geometry from renderer that doesn't belong to renderer
  // should simply report 0.
  EXPECT_EQ(geometry_state_.RemoveFromRenderer(other_renderer_name, source_id_,
                                               remove_id),
            0);
  confirm_renderers(removed_ids);

  // Tests for documented exception throwing.

  // Case: Source id does not map to a registered source.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveFromRenderer(other_renderer_name,
                                         SourceId::get_new_id(), remove_id),
      std::logic_error, "Referenced geometry source .* is not registered.");

  // Case: GeometryId does not map to a registered geometry.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveFromRenderer(other_renderer_name, source_id_,
                                         GeometryId::get_new_id()),
      std::logic_error, "Referenced geometry .* has not been registered.");

  // Case: GeometryId does not belong to SourceId.
  const SourceId source_id_2 =
      geometry_state_.RegisterNewSource("second_source");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveFromRenderer(other_renderer_name, source_id_2,
                                         remove_id),
      std::logic_error,
      "Trying to remove geometry \\d+ from the renderer '.+', but the geometry "
      "doesn't belong to given source .+");
}

// The frame-centric version of the RemoveGeometryFromRenderer test.
TEST_F(GeometryStateTest, RemoveFrameFromRenderer) {
  // TODO(SeanCurtis-TRI): Consider refactoring this set-up code between _this_
  // test and the RemoveGeometryFromRenderer test.
  const string other_renderer_name = "alt_renderer";
  geometry_state_.AddRenderer(other_renderer_name,
                              make_unique<DummyRenderEngine>());
  SetUpSingleSourceTree(Assign::kPerception);

  // Each geometry must have a render index for the default render engine. In
  // addition,
  //   a) have an index for the "other" render engine, xor
  //   b) be present in the `removed_from_renderer` set.
  auto confirm_renderers = [=](set<GeometryId> removed_from_renderer) {
    set<GeometryId> ids(geometries_.begin(), geometries_.end());
    ids.insert(anchored_geometry_);
    for (GeometryId id : ids) {
      // All should have render indices in the dummy renderer.
      EXPECT_TRUE(gs_tester_.GetGeometry(id)
                      ->render_index(kDummyRenderName)
                      .has_value());
      // Should have a render index if it is *not* in the removed set.
      EXPECT_EQ(gs_tester_.GetGeometry(id)
                    ->render_index(other_renderer_name)
                    .has_value(),
                removed_from_renderer.count(id) == 0);
    }
  };
  set<GeometryId> removed_ids;

  // Confirm geometries assigned to _both_ renderers.
  confirm_renderers(removed_ids);

  // Case: Remove a frame with multiple geometries registered with the renderer.
  EXPECT_EQ(geometry_state_.RemoveFromRenderer(other_renderer_name, source_id_,
                                               frames_[0]),
            2);
  // Geometries 0 & 1 are the known children of frame 0.
  removed_ids.insert(geometries_[0]);
  removed_ids.insert(geometries_[1]);
  confirm_renderers(removed_ids);

  // Case: Remove a frame with no geometries registered with the renderer.
  EXPECT_EQ(geometry_state_.RemoveFromRenderer(other_renderer_name, source_id_,
                                               frames_[0]),
            0);
  confirm_renderers(removed_ids);

  // Case: Remove a frame with *some* of the geometries registered with the
  // renderer. Achieve this by directly removing one of the child geometries.
  geometry_state_.RemoveFromRenderer(other_renderer_name, source_id_,
                                     geometries_[2]);
  removed_ids.insert(geometries_[2]);
  confirm_renderers(removed_ids);
  // Now remove the parent frame.
  EXPECT_EQ(geometry_state_.RemoveFromRenderer(other_renderer_name, source_id_,
                                               frames_[1]),
            1);
  removed_ids.insert(geometries_[3]);
  confirm_renderers(removed_ids);

  // Case: Source with no registered anchored geometry removing from world
  // frame.
  const SourceId source_id_2 =
      geometry_state_.RegisterNewSource("second_source");
  EXPECT_EQ(geometry_state_.RemoveFromRenderer(other_renderer_name, source_id_2,
                                               InternalFrame::world_frame_id()),
            0);
  confirm_renderers(removed_ids);

  // Case: Source with registered anchored geometry removing from world frame.
  EXPECT_EQ(geometry_state_.RemoveFromRenderer(other_renderer_name, source_id_,
                                               InternalFrame::world_frame_id()),
            1);
  removed_ids.insert(anchored_geometry_);
  confirm_renderers(removed_ids);

  // Tests for documented exception throwing.

  // Case: Source id does not map to a registered source.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveFromRenderer(other_renderer_name,
                                         SourceId::get_new_id(), frames_[0]),
      std::logic_error, "Referenced geometry source .* is not registered.");

  // Case: Frame does not map to a registered frame.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveFromRenderer(other_renderer_name, source_id_,
                                         FrameId::get_new_id()),
      std::logic_error,
      "Referenced frame .* but the frame doesn't belong to the source.");

  // Case: FrameId does not belong to SourceId.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveFromRenderer(other_renderer_name, source_id_2,
                                         frames_[0]),
      std::logic_error,
      "Referenced frame .+ but the frame doesn't belong to the source.");
}

// Successful invocations of AddRenderer are implicit in SetupSingleSource().
// This merely tests the error conditions.
TEST_F(GeometryStateTest, AddRendererError) {
  const string kName = "unique";
  EXPECT_NO_THROW(
      geometry_state_.AddRenderer(kName, make_unique<DummyRenderEngine>()));

  // Non-unique name.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.AddRenderer(kName, make_unique<DummyRenderEngine>()),
      std::logic_error,
      fmt::format("AddRenderer..: A renderer with the name '{}' already exists",
                  kName));

  // Geometry has been registered.
  SetUpSingleSourceTree();
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.AddRenderer(kName, make_unique<DummyRenderEngine>()),
      std::logic_error,
      fmt::format("AddRenderer..: Error adding renderer '{}'; geometries have "
                  "already been registered",
                  kName));
}

TEST_F(GeometryStateTest, GetRenderEngine) {
  const render::RenderEngine& engine =
      gs_tester_.GetRenderEngineOrThrow(kDummyRenderName);
  EXPECT_EQ(&engine, render_engine_);
  DRAKE_EXPECT_THROWS_MESSAGE(gs_tester_.GetRenderEngineOrThrow("bad name"),
                              std::logic_error,
                              "No renderer exists with name.*");
}

// Confirms that the renderer(s) have poses updated properly when
// FinalizePoseUpdate() is called.
TEST_F(GeometryStateTest, RendererPoseUpdate) {
  // Add a *second* render engine to make sure that *all* get updated.
  auto render_engine = make_unique<DummyRenderEngine>();
  DummyRenderEngine* second_engine = render_engine.get();
  geometry_state_.AddRenderer("second_engine", move(render_engine));

  SetUpSingleSourceTree(Assign::kPerception);

  // Reality check -- the render indices for each geometry should be the same
  // for each renderer.
  std::vector<RenderIndex> render_indices;
  for (int i = 0; i < single_tree_dynamic_geometry_count(); ++i) {
    const InternalGeometry* geometry = gs_tester_.GetGeometry(geometries_[i]);
    ASSERT_EQ(geometry->render_index(kDummyRenderName),
              geometry->render_index("second_engine"));
    render_indices.push_back(*geometry->render_index(kDummyRenderName));
  }

  EXPECT_EQ(second_engine->updated_indices().size(), 0u);
  EXPECT_EQ(render_engine_->updated_indices().size(), 0u);

  // Set poses of frames to the initial values.
  FramePoseVector<double> poses;
  for (int f = 0; f < static_cast<int>(frames_.size()); ++f) {
    poses.set_value(frames_[f], X_PFs_[f]);
  }
  gs_tester_.SetFramePoses(source_id_, poses);
  gs_tester_.FinalizePoseUpdate();

  // Confirm poses.
  auto expect_poses = [](const auto& test, const auto& expected) {
    EXPECT_EQ(test.size(), expected.size());
    for (const auto& expected_pair : expected) {
      const auto& test_iter = test.find(expected_pair.first);
      EXPECT_NE(test_iter, test.end());
      EXPECT_TRUE(CompareMatrices(test_iter->second.matrix(),
                                  expected_pair.second.matrix()));
    }
  };

  auto get_expected_indices = [this, &render_indices]() {
    map<RenderIndex, Isometry3d> expected;
    for (int i = 0; i < single_tree_dynamic_geometry_count(); ++i) {
      expected.emplace(render_indices[i],
                       gs_tester_.get_geometry_world_poses()[i]);
    }
    return expected;
  };

  map<RenderIndex, Isometry3d> expected_indices = get_expected_indices();
  expect_poses(second_engine->updated_indices(), expected_indices);
  expect_poses(render_engine_->updated_indices(), expected_indices);
  render_engine_->reset();
  second_engine->reset();

  // Set poses of frames to an alternate value - fixed offset from initial
  // values.
  const Vector3d offset{1, 2, 3};
  for (int f = 0; f < static_cast<int>(frames_.size()); ++f) {
    Isometry3d X_PF = X_PFs_[f];
    X_PF.translation() += offset;
    poses.set_value(frames_[f], X_PF);
  }
  EXPECT_EQ(second_engine->updated_indices().size(), 0u);
  EXPECT_EQ(render_engine_->updated_indices().size(), 0u);
  gs_tester_.SetFramePoses(source_id_, poses);
  gs_tester_.FinalizePoseUpdate();

  // Confirm poses.
  expected_indices = get_expected_indices();
  expect_poses(second_engine->updated_indices(), expected_indices);
  expect_poses(render_engine_->updated_indices(), expected_indices);
}

// The framework for testing the removal of roles, generally, parameterized on
// the role type.
class RemoveRoleTests : public GeometryStateTestBase,
                        public testing::TestWithParam<Role> {
 protected:
  void SetUp() {
    TestInit();
    // Add an *additional* renderer just to confirm that removing the perception
    // role from a geometry/frame still counts geometries removed and not
    // (geometry, renderer) pairs.
    geometry_state_.AddRenderer("other", make_unique<DummyRenderEngine>());
    SetUpSingleSourceTree(Assign::kPerception | Assign::kProximity |
                          Assign::kIllustration);
  }

  // Utility function to facilitate test. It asserts that *all* geometries have
  // all roles, _except_ the geometries whose ids are in the given set are
  // missing the indicated `role`.
  void ExpectAllRolesExcept(const set<GeometryId>& ids_without_role,
                            Role role_to_remove) const {
    for (GeometryId id : geometries_) {
      for (Role role :
           {Role::kProximity, Role::kIllustration, Role::kPerception}) {
        if (role == role_to_remove) {
          // If this id is *not* in the set with the role removed, then it
          // _should_ report as having the role.
          EXPECT_EQ(gs_tester_.GetGeometry(id)->has_role(role),
                    ids_without_role.count(id) == 0);
        } else {
          EXPECT_TRUE(gs_tester_.GetGeometry(id)->has_role(role));
        }
      }
    }
  }

  // The parameterized test for systematically testing the removal of the given
  // `role` while determining that all other roles remain untouched.
  void TestRemoveRoleFromGeometry(Role role_to_remove) {
    set<GeometryId> ids_without_role;

    // Relies on all geometries having all properties.
    ExpectAllRolesExcept(ids_without_role, role_to_remove);

    // Case: removing role from a single geometry reports removal.
    const InternalGeometry* geometry = gs_tester_.GetGeometry(geometries_[0]);
    EXPECT_TRUE(geometry->has_role(role_to_remove));
    EXPECT_EQ(
        geometry_state_.RemoveRole(source_id_, geometries_[0], role_to_remove),
        1);
    ids_without_role.insert(geometries_[0]);
    EXPECT_FALSE(geometry->has_role(role_to_remove));
    // Confirm change to removed role count and that other roles are
    // _unchanged_.
    ExpectAllRolesExcept(ids_without_role, role_to_remove);

    // Case: attempting to remove role from a geometry that has none has
    // no effect.
    EXPECT_EQ(
        geometry_state_.RemoveRole(source_id_, geometries_[0], role_to_remove),
        0);
    EXPECT_FALSE(geometry->has_role(role_to_remove));
    // Confirm role count still down one and that other roles are _unchanged_.
    ExpectAllRolesExcept(ids_without_role, role_to_remove);
  }

  void TestRemoveRoleFromFrame(Role role_to_remove) {
    set<GeometryId> ids_without_role;

    // Explicitly confirm that all geometries have all roles. In the balance of
    // this test, the only roles missing are the ones explicitly removed by this
    // test.
    ExpectAllRolesExcept(ids_without_role, role_to_remove);

    // Case: removing the role from the frame reports both geometries changed.
    ids_without_role.insert(geometries_[0]);
    ids_without_role.insert(geometries_[1]);
    EXPECT_EQ(
        geometry_state_.RemoveRole(source_id_, frames_[0], role_to_remove), 2);
    ExpectAllRolesExcept(ids_without_role, role_to_remove);

    // Case: attempting to remove role from the frame that has no geometries
    // with the role has no effect.
    EXPECT_EQ(
        geometry_state_.RemoveRole(source_id_, frames_[0], role_to_remove), 0);
    ExpectAllRolesExcept(ids_without_role, role_to_remove);

    // Case: Remove from frame when one geometry has the role and one geometry
    // does not. Remove the role from _one_ child geometry to set the initial
    // condition.
    geometry_state_.RemoveRole(source_id_, geometries_[2], role_to_remove);
    ids_without_role.insert(geometries_[2]);
    ExpectAllRolesExcept(ids_without_role, role_to_remove);

    // Invokes remove on the frame - only the single remaining geometry should
    // be affected.
    EXPECT_EQ(
        geometry_state_.RemoveRole(source_id_, frames_[1], role_to_remove), 1);
    ids_without_role.insert(geometries_[3]);
    ExpectAllRolesExcept(ids_without_role, role_to_remove);

    // Case: Operate on the world frame with a source that has no anchored
    // geometry. Should change nothing with no complaints.
    const SourceId source_id_2 = geometry_state_.RegisterNewSource("source2");
    EXPECT_EQ(geometry_state_.RemoveRole(
                  source_id_2, InternalFrame::world_frame_id(), role_to_remove),
              0);
    ExpectAllRolesExcept(ids_without_role, role_to_remove);

    // Case: Operate on the world frame with a source that *does* have anchored
    // geometry. Should remove the role from the single geometry.
    EXPECT_EQ(geometry_state_.RemoveRole(
                  source_id_, InternalFrame::world_frame_id(), role_to_remove),
              1);
    ids_without_role.insert(anchored_geometry_);
    ExpectAllRolesExcept(ids_without_role, role_to_remove);
  }
};

INSTANTIATE_TEST_CASE_P(GeometryStateTest, RemoveRoleTests,
                        ::testing::Values(Role::kProximity,
                                          Role::kIllustration,
                                          Role::kPerception));

TEST_P(RemoveRoleTests, RemoveRoleFromGeometry) {
  TestRemoveRoleFromGeometry(GetParam());
}

TEST_P(RemoveRoleTests, RemoveRoleFromFrame) {
  TestRemoveRoleFromFrame(GetParam());
}

// Tests that exceptions are thrown under the documented circumstances for
// removing roles.
TEST_F(RemoveRoleTests, RemoveRoleExceptions) {
  const SourceId invalid_source_id = SourceId::get_new_id();
  const FrameId invalid_frame_id = FrameId::get_new_id();
  const GeometryId invalid_geometry_id = GeometryId::get_new_id();

  // Case: Invalid source id (frame/geometry id and role don't matter).
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveRole(invalid_source_id, invalid_frame_id,
                                 Role::kUnassigned),
      std::logic_error, "Referenced geometry source \\d+ is not registered.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveRole(invalid_source_id, invalid_geometry_id,
                                 Role::kUnassigned),
      std::logic_error, "Referenced geometry source \\d+ is not registered.");

  // Case: valid source id, but invalid frame/geometry id.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveRole(source_id_, invalid_frame_id,
                                 Role::kUnassigned),
      std::logic_error, "Referenced .* frame doesn't belong to the source.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveRole(source_id_, invalid_geometry_id,
                                 Role::kUnassigned),

      std::logic_error, "Referenced geometry \\d+ has not been registered.");

  // Case: frame/geometry id belongs to a different source.
  const SourceId source_id_2 =
      geometry_state_.RegisterNewSource("second_source");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveRole(source_id_2, frames_[0],
                                 Role::kUnassigned),
      std::logic_error, "Referenced .* frame doesn't belong to the source.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveRole(source_id_2, geometries_[0],
                                 Role::kUnassigned),
      std::logic_error, ".*the geometry doesn't belong to that source.");
}

// Special version of the class that does *not* default to having a renderer.
class GeometryStateNoRendererTest : public GeometryStateTestBase,
                                    public ::testing::Test {
 protected:
  void SetUp() override {
    TestInit(false /* no renderer */);
    SetUpSingleSourceTree(Assign::kPerception);
  }
};

// Tests the special case of assigning/removing perception roles even when there
// is no renderer.
TEST_F(GeometryStateNoRendererTest, PerceptionRoleWithoutRenderer) {
  const InternalGeometry& geometry = *gs_tester_.GetGeometry(geometries_[0]);
  ASSERT_EQ(gs_tester_.render_engines().size(), 0u);
  EXPECT_TRUE(geometry.has_perception_role());

  EXPECT_EQ(
      geometry_state_.RemoveRole(source_id_, geometries_[0], Role::kPerception),
      1);
  EXPECT_FALSE(geometry.has_perception_role());
  EXPECT_EQ(
      geometry_state_.RemoveRole(source_id_, geometries_[0], Role::kPerception),
      0);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
