#include "drake/geometry/geometry_state.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/geometry_set.h"
#include "drake/geometry/geometry_version.h"
#include "drake/geometry/internal_frame.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/render/render_label.h"
#include "drake/geometry/shape_specification.h"
#include "drake/geometry/test_utilities/dummy_render_engine.h"

namespace drake {
namespace geometry {

using Eigen::Translation3d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using internal::DrivenTriangleMesh;
using internal::DummyRenderEngine;
using internal::FrameNameSet;
using internal::HydroelasticType;
using internal::InternalFrame;
using internal::InternalGeometry;
using internal::kComplianceType;
using internal::kElastic;
using internal::kFriction;
using internal::kHcDissipation;
using internal::kHydroGroup;
using internal::kMaterialGroup;
using internal::kPointStiffness;
using internal::kRelaxationTime;
using internal::kRezHint;
using internal::kSlabThickness;
using internal::ProximityEngine;
using math::RigidTransform;
using math::RigidTransformd;
using render::RenderLabel;
using std::make_unique;
using std::map;
using std::optional;
using std::pair;
using std::set;
using std::string;
using std::unique_ptr;
using std::unordered_map;
using std::unordered_set;
using std::vector;

template <typename T>
using IdPoseMap = unordered_map<GeometryId, RigidTransform<T>>;
template <typename T>
using IdConfigurationMap = unordered_map<GeometryId, VectorX<T>>;

// Implementation of friend class that allows me to peek into the geometry state
// to confirm invariants on the state's internal workings as a result of
// operations.

template <class T>
class GeometryStateTester {
 public:
  void set_state(GeometryState<T>* state) { state_ = state; }

  FrameId get_world_frame() const { return InternalFrame::world_frame_id(); }

  SourceId get_self_source_id() const { return state_->self_source_; }

  const unordered_map<SourceId, string>& get_source_name_map() const {
    return state_->source_names_;
  }

  const unordered_map<SourceId, FrameIdSet>& get_source_frame_id_map() const {
    return state_->source_frame_id_map_;
  }

  const unordered_map<SourceId, FrameNameSet>& get_source_frame_name_map()
      const {
    return state_->source_frame_name_map_;
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

  const vector<FrameId>& get_frame_index_id_map() const {
    return state_->frame_index_to_id_map_;
  }

  const IdPoseMap<T>& get_geometry_world_poses() const {
    return state_->kinematics_data_.X_WGs;
  }

  const IdConfigurationMap<T>& get_geometry_world_configurations() const {
    return state_->kinematics_data_.q_WGs;
  }

  const vector<RigidTransform<T>>& get_frame_world_poses() const {
    return state_->kinematics_data_.X_WFs;
  }

  const vector<RigidTransform<T>>& get_frame_parent_poses() const {
    return state_->kinematics_data_.X_PFs;
  }

  internal::KinematicsData<T>& mutable_kinematics_data() const {
    return state_->mutable_kinematics_data();
  }

  const internal::DrivenMeshData& driven_mesh_data(Role role) const {
    return state_->kinematics_data_.driven_mesh_data.at(role);
  }

  void SetFramePoses(SourceId source_id, const FramePoseVector<T>& poses,
                     internal::KinematicsData<T>* kinematics_data) {
    state_->SetFramePoses(source_id, poses, kinematics_data);
  }

  void SetGeometryConfiguration(
      SourceId source_id, const GeometryConfigurationVector<T>& configuration,
      internal::KinematicsData<T>* kinematics_data) {
    state_->SetGeometryConfiguration(source_id, configuration, kinematics_data);
  }

  void FinalizePoseUpdate() {
    state_->FinalizePoseUpdate(state_->kinematics_data_,
                               &state_->mutable_proximity_engine(),
                               state_->GetMutableRenderEngines());
  }

  void FinalizeConfigurationUpdate() {
    for (const auto role : std::vector<Role>{
             Role::kIllustration, Role::kPerception, Role::kProximity}) {
      state_->kinematics_data_.driven_mesh_data[role].SetControlMeshPositions(
          state_->kinematics_data_.q_WGs);
    }
    state_->FinalizeConfigurationUpdate(state_->kinematics_data_,
                                        &state_->mutable_proximity_engine(),
                                        state_->GetMutableRenderEngines());
  }

  template <typename ValueType>
  void ValidateFrameIds(
      SourceId source_id,
      const KinematicsVector<FrameId, ValueType>& data) const {
    state_->ValidateFrameIds(source_id, data);
  }

  const InternalGeometry* GetGeometry(GeometryId id) const {
    return state_->GetGeometry(id);
  }

  const render::RenderEngine& GetRenderEngineOrThrow(
      const std::string& name) const {
    return state_->GetRenderEngineOrThrow(name);
  }

  const ProximityEngine<T>& proximity_engine() const {
    return *state_->geometry_engine_;
  }

  int RendererCount() const { return state_->RendererCount(); }

  const GeometryVersion& geometry_version() const {
    return state_->geometry_version_;
  }

  // Provides acesss to the scalar-converting copy constructor.
  template <typename U>
  static std::unique_ptr<GeometryState<T>> CopyGeometryState(
      const GeometryState<U>& other) {
    return std::unique_ptr<GeometryState<T>>(new GeometryState<T>(other));
  }

 private:
  GeometryState<T>* state_;
};

namespace internal {

class ProximityEngineTester {
 public:
  static const hydroelastic::Geometries& hydroelastic_geometries(
      const ProximityEngine<double>& engine) {
    return engine.hydroelastic_geometries();
  }
};

}  // namespace internal

namespace {

namespace fs = std::filesystem;

static fs::path temp_dir() {
  static const never_destroyed<fs::path> temp_path(temp_directory());
  return temp_path.access();
}

fs::path WriteFile(std::string_view contents, std::string_view file_name) {
  const fs::path file_path = temp_dir() / file_name;
  std::ofstream out(file_path);
  DRAKE_DEMAND(out.is_open());
  out << contents;
  return file_path;
}

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
        make_unique<GeometryInstance>(RigidTransformd::Identity(),
                                      make_unique<ShapeType>(expected_),
                                      "shape"));
    state->GetShape(g_id).Reify(this);
    return result_;
  }

  // Shape reifier implementations.
  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Box& box, void*) final {
    if (IsExpectedType(box)) {
      TestShapeParameters(box);
    }
  }

  void ImplementGeometry(const Capsule& capsule, void*) final {
    if (IsExpectedType(capsule)) {
      TestShapeParameters(capsule);
    }
  }

  void ImplementGeometry(const Convex& convex, void*) final {
    if (IsExpectedType(convex)) {
      TestShapeParameters(convex);
    }
  }

  void ImplementGeometry(const Cylinder& cylinder, void*) final {
    if (IsExpectedType(cylinder)) {
      TestShapeParameters(cylinder);
    }
  }

  // TODO(SeanCurtis-TRI): Why are we missing ellipsoid and meshcat cone?

  void ImplementGeometry(const HalfSpace& half_space, void*) final {
    // Halfspace has no parameters; so no further testing is necessary.
    IsExpectedType(half_space);
  }

  void ImplementGeometry(const Mesh& mesh, void*) final {
    if (IsExpectedType(mesh)) {
      TestShapeParameters(mesh);
    }
  }

  void ImplementGeometry(const Sphere& sphere, void*) final {
    if (IsExpectedType(sphere)) {
      TestShapeParameters(sphere);
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
  if (test.radius() != expected_.radius()) {
    error() << "\nExpected sphere radius " << expected_.radius() << ", "
            << "received sphere radius " << test.radius();
  }
}

template <>
template <>
void ShapeMatcher<Cylinder>::TestShapeParameters(const Cylinder& test) {
  if (test.radius() != expected_.radius()) {
    error() << "\nExpected cylinder radius " << expected_.radius() << ", "
            << "received cylinder radius " << test.radius();
  }
  if (test.length() != expected_.length()) {
    error() << "\nExpected cylinder length " << expected_.length()
            << ", received cylinder length " << test.length();
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
void ShapeMatcher<Capsule>::TestShapeParameters(const Capsule& test) {
  if (test.radius() != expected_.radius()) {
    error() << "\nExpected capsule radius " << expected_.radius() << ", "
            << "received capsule radius " << test.radius();
  }
  if (test.length() != expected_.length()) {
    error() << "\nExpected capsule length " << expected_.length()
            << ", received capsule length " << test.length();
  }
}

template <>
template <>
void ShapeMatcher<Mesh>::TestShapeParameters(const Mesh& test) {
  if (test.source().description() != expected_.source().description()) {
    error() << "\nExpected description " << expected_.source().description()
            << ", received description " << test.source().description();
  }
  // Looking for *exact* match.
  if (test.scale3() != expected_.scale3()) {
    error() << "\nExpected mesh scale "
            << fmt::to_string(fmt_eigen(expected_.scale3().transpose()))
            << ", received mesh scale "
            << fmt::to_string(fmt_eigen(test.scale3().transpose()));
  }
}

template <>
template <>
void ShapeMatcher<Convex>::TestShapeParameters(const Convex& test) {
  if (test.source().description() != expected_.source().description()) {
    error() << "\nExpected description " << expected_.source().description()
            << ", received description " << test.source().description();
  }
  // Looking for *exact* match.
  if (test.scale3() != expected_.scale3()) {
    error() << "\nExpected convex scale "
            << fmt::to_string(fmt_eigen(expected_.scale3().transpose()))
            << ", received convex scale "
            << fmt::to_string(fmt_eigen(test.scale3().transpose()));
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
    instance_pose_.set_translation({10, 20, 30});
    instance_ = make_unique<GeometryInstance>(
        instance_pose_, make_unique<Sphere>(1.0), "instance");
    gs_tester_.set_state(&geometry_state_);
    if (add_renderer) {
      auto render_engine = make_unique<DummyRenderEngine>();
      render_engine_ = render_engine.get();
      geometry_state_.AddRenderer(kDummyRenderName, std::move(render_engine));
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
    // 90° around x-axis.
    RigidTransformd pose(AngleAxis<double>(M_PI_2, Vector3d::UnitX()),
                         Vector3d{1, 2, 3});
    frames_.push_back(
        geometry_state_.RegisterFrame(source_id_, GeometryFrame("f0")));
    X_WFs_.push_back(pose);
    X_PFs_.push_back(pose);

    // Create f1.
    // 90° around y-axis.
    pose = RigidTransformd(AngleAxis<double>(M_PI_2, Vector3d::UnitY()),
                           Vector3d{10, 20, 30});
    frames_.push_back(
        geometry_state_.RegisterFrame(source_id_, GeometryFrame("f1")));
    X_WFs_.push_back(pose);
    X_PFs_.push_back(pose);

    // Create f2.
    pose = pose.inverse();
    frames_.push_back(geometry_state_.RegisterFrame(source_id_, frames_[1],
                                                    GeometryFrame("f2")));
    X_WFs_.push_back(X_WFs_[1] * pose);
    X_PFs_.push_back(pose);

    // Add geometries to each frame.
    const Vector3<double> x_axis(1, 0, 0);
    geometries_.resize(kFrameCount * kGeometryCount);
    geometry_names_.resize(geometries_.size());
    int g_count = 0;
    for (auto frame_id : frames_) {
      for (int i = 0; i < kGeometryCount; ++i) {
        pose = RigidTransformd(AngleAxis<double>(g_count * M_PI_2, x_axis),
                               Vector3d{g_count + 1.0, 0, 0});
        // Have the name reflect the frame and the index in the geometry.
        const string& name = to_string(frame_id) + "_g" + to_string(i);
        geometry_names_[g_count] = name;
        geometries_[g_count] = geometry_state_.RegisterGeometry(
            source_id_, frame_id,
            make_unique<GeometryInstance>(pose, make_unique<Sphere>(1), name));
        X_FGs_.push_back(pose);
        ++g_count;
      }
    }

    // Create anchored geometry.
    X_WA_ = RigidTransformd{Translation3d{0, 0, -1}};
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

  template <typename PropertyType>
  void AssignRoleToSingleSourceTree(const PropertyType& properties) {
    ASSERT_TRUE(source_id_.is_valid());
    for (GeometryId id : geometries_) {
      geometry_state_.AssignRole(source_id_, id, properties);
    }
    for (GeometryId id : deformable_geometries_) {
      geometry_state_.AssignRole(source_id_, id, properties);
    }
    geometry_state_.AssignRole(source_id_, anchored_geometry_, properties);
  }

  // This function sets up a dummy GeometryState that contains both rigid
  // (non-deformable) and deformable geometries to facilitate testing.
  SourceId SetUpWithRigidAndDeformableGeometries(
      Assign roles_to_assign = Assign::kNone) {
    SourceId s_id = SetUpSingleSourceTree();
    const Sphere sphere(1.0);
    constexpr double kRezHint = 0.5;
    auto instance = make_unique<GeometryInstance>(
        RigidTransformd::Identity(), make_unique<Sphere>(sphere), "sphere");
    GeometryId g_id = geometry_state_.RegisterDeformableGeometry(
        s_id, InternalFrame::world_frame_id(), std::move(instance), kRezHint);
    deformable_geometries_.push_back(g_id);
    if ((roles_to_assign & Assign::kProximity) != Assign::kNone) {
      AssignProximityToSingleSourceTree();
    }
    if ((roles_to_assign & Assign::kPerception) != Assign::kNone) {
      AssignPerceptionToSingleSourceTree();
    }
    if ((roles_to_assign & Assign::kIllustration) != Assign::kNone) {
      AssignIllustrationToSingleSourceTree();
    }
    return s_id;
  }

  // Reports characteristics of the dummy tree.
  int single_tree_frame_count() const {
    // Added dynamic frames plus the world frame.
    return kFrameCount + 1;
  }

  // Total number of rigid geometries.
  int single_tree_rigid_geometry_count() const {
    return single_tree_dynamic_rigid_geometry_count() +
           anchored_geometry_count();
  }

  // Total number of dynamic rigid geometries.
  int single_tree_dynamic_rigid_geometry_count() const {
    return kFrameCount * kGeometryCount;
  }

  int total_geometry_count() const {
    return single_tree_rigid_geometry_count() + ssize(deformable_geometries_);
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
  RigidTransformd instance_pose_{RigidTransformd::Identity()};
  GeometryState<double> geometry_state_;
  GeometryStateTester<double> gs_tester_;
  DummyRenderEngine* render_engine_{};

  // Values for setting up and testing the dummy tree.
  enum Counts {
    kFrameCount = 3,
    kGeometryCount = 2  // Geometries *per frame*.
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
  // All deformable geometry ids.
  vector<GeometryId> deformable_geometries_;
  // The registered name of the anchored geometry.
  const string anchored_name_{"anchored"};
  // The id of the single-source tree.
  SourceId source_id_;

  // The poses of the frames in the world frame.
  vector<RigidTransformd> X_WFs_;
  // The poses of the frames in the parent's frame.
  vector<RigidTransformd> X_PFs_;
  // The poses of the dynamic geometries in the parent frame.
  vector<RigidTransformd> X_FGs_;
  // The pose of the anchored geometry in the world frame.
  RigidTransformd X_WA_;
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
    // Add an arbitrary, meaningless property so we can detect if the properties
    // get thrown out.
    properties.AddProperty("test", "value", 17);
    AssignRoleToSingleSourceTree(properties);
  }

  // Convenience method for assigning illustration properties to all geometries
  // in the single source tree.
  void AssignIllustrationToSingleSourceTree() {
    ASSERT_TRUE(source_id_.is_valid());
    IllustrationProperties properties;
    properties.AddProperty("phong", "diffuse",
                           Vector4<double>{0.8, 0.8, 0.8, 1.0});
    // Add an arbitrary, meaningless property so we can detect if the properties
    // get thrown out.
    properties.AddProperty("test", "value", 17);
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
    // Add an arbitrary, meaningless property so we can detect if the properties
    // get thrown out.
    properties.AddProperty("test", "value", 17);
    AssignRoleToSingleSourceTree(properties);
  }
};

// Class for performing most tests on GeometryState. It does *not* populate the
// GeometryState at all, but relies on each test to do so as appropriate
// (via the SetUpSingleSourceTree() method).
class GeometryStateTest : public GeometryStateTestBase, public ::testing::Test {
 protected:
  void SetUp() override { TestInit(); }

  // Helpers for forwarding member methods of Geometry State and verifying
  // behaviors of Geometry Versions.
  //
  // The helpers are templatized on the return type of the method and two sets
  // of arguments, DeclaredArgs and GivenArgs. While generally one would
  // assume the two sets of args are of the same type, separating them this
  // way facilitates type deduction. For example, passing in a string literal
  // as a parameter to a function that expects a const string would defy type
  // deduction with just a single argument template. But separating them at
  // the call to this function, defers the resolution to the actual call of
  // the function.

  // Forward a function call to a member method of GeometryState and return the
  // GeometryVersions before and after the call.
  template <typename ReturnType, typename... DeclaredArgs,
            typename... GivenArgs>
  std::tuple<GeometryVersion, GeometryVersion> ForwardAndGetRevisions(
      ReturnType (GeometryState<double>::*f)(DeclaredArgs...),
      GivenArgs&&... args) {
    GeometryVersion old_version = geometry_state_.geometry_version();
    (geometry_state_.*f)(std::forward<GivenArgs>(args)...);
    GeometryVersion new_version = geometry_state_.geometry_version();
    return {old_version, new_version};
  }

  // Forward a call to a member function in GeometryState and verify that the
  // version of the given `role` is modified but all other versions are
  // unchanged in this function.
  template <typename ReturnType, typename... DeclaredArgs,
            typename... GivenArgs>
  void VerifyRoleVersionModified(
      Role role, ReturnType (GeometryState<double>::*f)(DeclaredArgs...),
      GivenArgs&&... args) {
    auto [old_version, new_version] =
        ForwardAndGetRevisions(f, std::forward<GivenArgs>(args)...);
    for (Role test_role :
         {Role::kProximity, Role::kPerception, Role::kIllustration}) {
      // We expect versions to match when the test_role is different from the
      // declared role.
      EXPECT_EQ(old_version.IsSameAs(new_version, test_role),
                role != test_role);
    }
  }

  // Forward a call to a non-void member function in GeometryState and verify
  // that all versions are unchanged in this function. Return the return
  // value of the forwarded call.
  template <typename ReturnType, typename... DeclaredArgs,
            typename... GivenArgs>
  ReturnType VerifyVersionUnchanged(
      ReturnType (GeometryState<double>::*f)(DeclaredArgs...),
      GivenArgs&&... args) {
    GeometryVersion old_version = geometry_state_.geometry_version();
    ReturnType ret = (geometry_state_.*f)(std::forward<GivenArgs>(args)...);
    GeometryVersion new_version = geometry_state_.geometry_version();
    EXPECT_TRUE(old_version.IsSameAs(new_version, Role::kProximity));
    EXPECT_TRUE(old_version.IsSameAs(new_version, Role::kPerception));
    EXPECT_TRUE(old_version.IsSameAs(new_version, Role::kIllustration));
    return ret;
  }

  // Forward a call to a void member function in GeometryState and verify that
  // all versions are unchanged in this function.
  template <typename... DeclaredArgs, typename... GivenArgs>
  void VerifyVersionUnchanged(void (GeometryState<double>::*f)(DeclaredArgs...),
                              GivenArgs&&... args) {
    auto [old_version, new_version] =
        ForwardAndGetRevisions(f, std::forward<GivenArgs>(args)...);
    EXPECT_TRUE(old_version.IsSameAs(new_version, Role::kProximity));
    EXPECT_TRUE(old_version.IsSameAs(new_version, Role::kPerception));
    EXPECT_TRUE(old_version.IsSameAs(new_version, Role::kIllustration));
  }

  void VerifyIdenticalVersions(const GeometryState<double>& gs1,
                               const GeometryState<double>& gs2) const {
    const auto& v1 = gs1.geometry_version();
    const auto& v2 = gs2.geometry_version();
    EXPECT_TRUE(v1.IsSameAs(v2, Role::kProximity));
    EXPECT_TRUE(v1.IsSameAs(v2, Role::kPerception));
    EXPECT_TRUE(v1.IsSameAs(v2, Role::kIllustration));
  }
};

// Confirms that a new GeometryState has no data.
TEST_F(GeometryStateTest, Constructor) {
  // GeometryState has a "self source".
  EXPECT_EQ(geometry_state_.get_num_sources(), 1);
  // GeometryState always has a world frame.
  EXPECT_EQ(geometry_state_.get_num_frames(), 1);
  EXPECT_EQ(geometry_state_.get_num_geometries(), 0);
  EXPECT_EQ(gs_tester_.get_source_frame_name_map()
                .find(gs_tester_.get_self_source_id())
                ->second,
            internal::FrameNameSet{"world"});
}

// Confirms that the registered shapes are correctly returned upon
// introspection.
TEST_F(GeometryStateTest, IntrospectShapes) {
  const SourceId source_id = geometry_state_.RegisterNewSource("test_source");
  const FrameId frame_id =
      geometry_state_.RegisterFrame(source_id, GeometryFrame("frame"));

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
    ShapeMatcher<Capsule> matcher(Capsule(0.25, 2.0));
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
      "No geometry available for invalid geometry id: .+");
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
  DRAKE_EXPECT_NO_THROW((s_id = geometry_state_.RegisterNewSource(name)));
  EXPECT_TRUE(geometry_state_.SourceIsRegistered(s_id));
  EXPECT_EQ(geometry_state_.GetName(s_id), name);

  // The first source id is the internal source.
  ASSERT_EQ(geometry_state_.GetAllSourceIds().size(), 2);
  const SourceId world_source = geometry_state_.GetAllSourceIds().front();
  EXPECT_THAT(geometry_state_.FramesForSource(world_source),
              testing::ElementsAre(gs_tester_.get_world_frame()));

  // The second source id is the one we added.
  EXPECT_EQ(geometry_state_.GetAllSourceIds().back(), s_id);

  // Case: User-specified name duplicates previously registered name.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterNewSource(name),
      "Registering new source with duplicate name: Unique.");

  // Case: query with invalid source id.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.GetName(SourceId::get_new_id()),
      "Querying source name for an invalid source id: \\d+.");
}

// Tests the geometry statistics values. It uses the single-source tree to
// create a state with interesting metrics. Also confirms the "is registered"
// -ness of known valid sources and known invalid sources.
TEST_F(GeometryStateTest, GeometryStatistics) {
  const SourceId dummy_source = SetUpSingleSourceTree();

  EXPECT_TRUE(geometry_state_.SourceIsRegistered(dummy_source));
  // Dummy source + self source.
  EXPECT_EQ(geometry_state_.get_num_sources(), 2);
  EXPECT_EQ(geometry_state_.get_num_frames(), single_tree_frame_count());
  EXPECT_EQ(geometry_state_.NumFramesForSource(source_id_),
            single_tree_frame_count() - 1);  // subtract the world frame.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.NumFramesForSource(SourceId::get_new_id()),
      "Referenced geometry source .* is not registered.");
  EXPECT_EQ(geometry_state_.NumDynamicGeometries(),
            single_tree_dynamic_rigid_geometry_count());
  EXPECT_EQ(geometry_state_.NumAnchoredGeometries(), anchored_geometry_count());
  EXPECT_EQ(
      geometry_state_.NumAnchoredGeometries(),
      geometry_state_.NumGeometriesForFrame(InternalFrame::world_frame_id()));
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            single_tree_rigid_geometry_count());
  const SourceId false_id = SourceId::get_new_id();
  EXPECT_FALSE(geometry_state_.SourceIsRegistered(false_id));
}

TEST_F(GeometryStateTest, GetOwningSourceName) {
  SetUpSingleSourceTree();

  EXPECT_EQ(kSourceName, geometry_state_.GetOwningSourceName(frames_[0]));
  EXPECT_EQ(kSourceName, geometry_state_.GetOwningSourceName(geometries_[0]));
  EXPECT_EQ(kSourceName,
            geometry_state_.GetOwningSourceName(anchored_geometry_));

  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.GetOwningSourceName(FrameId::get_new_id()),
      "Referenced frame .* has not been registered.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.GetOwningSourceName(GeometryId::get_new_id()),
      "Geometry id .* does not map to a registered geometry");
}

TEST_F(GeometryStateTest, RenameFrame) {
  SetUpSingleSourceTree();
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RenameFrame(FrameId::get_new_id(), "invalid"),
      ".*Cannot rename.*invalid frame id:.*");
  DRAKE_EXPECT_THROWS_MESSAGE(geometry_state_.RenameFrame(frames_[0], "f1"),
                              ".*already existing.*");

  // Renaming a frame to its existing name does not cause errors.
  std::string original_name0(geometry_state_.GetName(frames_[0]));
  geometry_state_.RenameFrame(frames_[0], original_name0);
  EXPECT_EQ(geometry_state_.GetName(frames_[0]), original_name0);

  geometry_state_.RenameFrame(frames_[0], "something");
  EXPECT_EQ(geometry_state_.GetName(frames_[0]), "something");
}

TEST_F(GeometryStateTest, RenameGeometry) {
  SetUpSingleSourceTree();
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RenameGeometry(GeometryId::get_new_id(), "invalid"),
      ".*Cannot rename.*invalid geometry id:.*");

  // Renaming a geometry to its existing name does not cause errors.
  std::string original_name0(geometry_state_.GetName(geometries_[0]));
  geometry_state_.RenameGeometry(geometries_[0], original_name0);
  EXPECT_EQ(geometry_state_.GetName(geometries_[0]), original_name0);

  geometry_state_.RenameGeometry(geometries_[0], "something");
  EXPECT_EQ(geometry_state_.GetName(geometries_[0]), "something");
}

// Compares the transmogrified geometry state (embedded in its tester) against
// the double state to confirm they have the same values/topology.
template <typename T>
void ExpectSuccessfulTransmogrification(
    const GeometryStateTester<T>& T_tester,
    const GeometryStateTester<double>& d_tester) {
  // 1. Test all of the identifier -> trivially testable value maps
  EXPECT_EQ(T_tester.get_self_source_id(), d_tester.get_self_source_id());
  EXPECT_EQ(T_tester.get_source_name_map(), d_tester.get_source_name_map());
  EXPECT_EQ(T_tester.get_source_frame_id_map(),
            d_tester.get_source_frame_id_map());
  EXPECT_EQ(T_tester.get_source_frame_name_map(),
            d_tester.get_source_frame_name_map());
  EXPECT_EQ(T_tester.get_source_root_frame_map(),
            d_tester.get_source_root_frame_map());
  EXPECT_EQ(T_tester.get_source_anchored_geometry_map(),
            d_tester.get_source_anchored_geometry_map());
  EXPECT_EQ(T_tester.get_geometries(), d_tester.get_geometries());
  EXPECT_EQ(T_tester.get_frames(), d_tester.get_frames());

  // 2. Confirm that the ids all made it across.
  EXPECT_EQ(T_tester.get_frame_index_id_map(),
            d_tester.get_frame_index_id_map());

  // 3. Compare RigidTransformd with RigidTransform<AutoDiffXd>.
  for (const auto& id_geom_pair : T_tester.get_geometries()) {
    const GeometryId id = id_geom_pair.first;
    const auto& T_geometry = id_geom_pair.second;
    EXPECT_TRUE(CompareMatrices(
        T_geometry.X_FG().GetAsMatrix34(),
        d_tester.get_geometries().at(id).X_FG().GetAsMatrix34()));
  }

  // 4. Compare RigidTransform<AutoDiffXd> with RigidTransformd.
  auto test_T_vs_double_pose = [](const RigidTransform<T>& test,
                                  const RigidTransformd& ref) {
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 4; ++col) {
        EXPECT_EQ(ExtractDoubleOrThrow(test.GetAsMatrix34()(row, col)),
                  ref.GetAsMatrix34()(row, col));
      }
    }
  };

  auto test_T_vs_double = [test_T_vs_double_pose](
                              const vector<RigidTransform<T>>& test,
                              const vector<RigidTransformd>& ref) {
    EXPECT_EQ(test.size(), ref.size());
    for (size_t i = 0; i < ref.size(); ++i) {
      test_T_vs_double_pose(test[i], ref[i]);
    }
  };

  auto test_T_vs_double_pose_map = [test_T_vs_double_pose](
                                       const IdPoseMap<T>& test,
                                       const IdPoseMap<double>& ref) {
    ASSERT_EQ(test.size(), ref.size());
    for (const auto& id_pose_pair : ref) {
      const GeometryId id = id_pose_pair.first;
      const RigidTransformd& ref_pose = id_pose_pair.second;
      test_T_vs_double_pose(test.at(id), ref_pose);
    }
  };

  auto test_T_vs_double_configuration_map =
      [](const IdConfigurationMap<T>& test,
         const IdConfigurationMap<double>& ref) {
        ASSERT_EQ(test.size(), ref.size());
        for (const auto& id_configuration_pair : ref) {
          const GeometryId id = id_configuration_pair.first;
          const VectorX<double>& ref_configuration =
              id_configuration_pair.second;
          EXPECT_EQ(ExtractDoubleOrThrow(test.at(id)), ref_configuration);
        }
      };

  test_T_vs_double(T_tester.get_frame_parent_poses(),
                   d_tester.get_frame_parent_poses());
  test_T_vs_double_pose_map(T_tester.get_geometry_world_poses(),
                            d_tester.get_geometry_world_poses());
  test_T_vs_double(T_tester.get_frame_world_poses(),
                   d_tester.get_frame_world_poses());
  test_T_vs_double_configuration_map(
      T_tester.get_geometry_world_configurations(),
      d_tester.get_geometry_world_configurations());

  // 5. Engine transmogrification tested in its own test; it will *not* be done
  // here.
}

// This tests the ability to assign a GeometryState<double> to a
// GeometryState<T>. Implicitly uses transmogrification.
TEST_F(GeometryStateTest, AssignDoubleToScalarType) {
  SetUpWithRigidAndDeformableGeometries();
  GeometryState<AutoDiffXd> ad_state{};
  ad_state = geometry_state_;
  GeometryStateTester<AutoDiffXd> ad_tester;
  ad_tester.set_state(&ad_state);
  ExpectSuccessfulTransmogrification(ad_tester, gs_tester_);

  GeometryState<symbolic::Expression> sym_state{};
  sym_state = geometry_state_;
  GeometryStateTester<symbolic::Expression> sym_tester;
  sym_tester.set_state(&sym_state);
  ExpectSuccessfulTransmogrification(sym_tester, gs_tester_);
}

// This tests the ability to copy a GeometryState<AutoDiffXd> to a
// GeometryState<double>.
TEST_F(GeometryStateTest, CopyAutoDiffIntoDouble) {
  // Prepare the non-double copies of the state.
  SetUpWithRigidAndDeformableGeometries();
  GeometryState<AutoDiffXd> ad_state;
  ad_state = geometry_state_;
  GeometryStateTester<AutoDiffXd> ad_tester;
  ad_tester.set_state(&ad_state);

  // Convert.
  std::unique_ptr<GeometryState<double>> dut =
      GeometryStateTester<double>::CopyGeometryState(ad_state);

  // Check for equality.
  GeometryStateTester<double> d_tester;
  d_tester.set_state(dut.get());
  ExpectSuccessfulTransmogrification(ad_tester, d_tester);
}

// This tests the ability to copy a GeometryState<Expression> to a
// GeometryState<double>.
TEST_F(GeometryStateTest, CopySymbolicIntoDouble) {
  // Prepare the non-double copies of the state.
  SetUpWithRigidAndDeformableGeometries();
  GeometryState<symbolic::Expression> sym_state;
  sym_state = geometry_state_;
  GeometryStateTester<symbolic::Expression> sym_tester;
  sym_tester.set_state(&sym_state);

  // Convert.
  std::unique_ptr<GeometryState<double>> dut =
      GeometryStateTester<double>::CopyGeometryState(sym_state);

  // Check for equality.
  GeometryStateTester<double> d_tester;
  d_tester.set_state(dut.get());
  ExpectSuccessfulTransmogrification(sym_tester, d_tester);
}

// Uses the single source tree to confirm that the data has migrated properly.
TEST_F(GeometryStateTest, Transmogrify) {
  SetUpWithRigidAndDeformableGeometries();
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
    const auto& f_id_set = s_f_id_map.at(s_id);
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
    const auto& root_id_set = s_root_map.at(s_id);
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

    auto test_frame = [internal_frames, this, s_id](int i, FrameId parent_id,
                                                    int num_child_frames) {
      const auto& frame = internal_frames.at(frames_[i]);
      EXPECT_EQ(frame.source_id(), s_id);
      EXPECT_EQ(frame.id(), frames_[i]);
      EXPECT_EQ(frame.name(), "f" + to_string(i));
      EXPECT_EQ(frame.frame_group(), 0);  // Defaults to zero.
      EXPECT_EQ(frame.index(), i + 1);    // ith frame added.
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
          CompareMatrices(frame_in_parent[frame.index()].GetAsMatrix34(),
                          X_PFs_[i].GetAsMatrix34()));
    };

    // When added, all frames' poses w.r.t. their parents are the identity.
    const auto& frame_in_parent = gs_tester_.get_frame_parent_poses();
    for (FrameId frame_id : frames_) {
      const auto& frame = internal_frames.at(frame_id);
      EXPECT_TRUE(
          CompareMatrices(frame_in_parent[frame.index()].GetAsMatrix34(),
                          RigidTransformd::Identity().GetAsMatrix34()));
    }

    // Confirm posing positions the frames properly.
    FramePoseVector<double> poses;
    for (int f = 0; f < static_cast<int>(frames_.size()); ++f) {
      poses.set_value(frames_[f], X_PFs_[f]);
    }
    gs_tester_.SetFramePoses(s_id, poses,
                             &gs_tester_.mutable_kinematics_data());
    gs_tester_.FinalizePoseUpdate();

    test_frame(0, gs_tester_.get_world_frame(), 0);
    test_frame(1, gs_tester_.get_world_frame(), 1);
    test_frame(2, frames_[1], 0);
  }

  // The internal geometries are what and where they should be.
  {
    const auto& internal_geometries = gs_tester_.get_geometries();
    EXPECT_EQ(internal_geometries.size(), single_tree_rigid_geometry_count());
    // NOTE: This relies on the anchored geometry being added *last*.
    for (int i = 0; i < single_tree_dynamic_rigid_geometry_count(); ++i) {
      const auto& geometry = internal_geometries.at(geometries_[i]);
      EXPECT_EQ(geometry.frame_id(), frames_[i / kGeometryCount]);
      EXPECT_EQ(geometry.id(), geometries_[i]);
      EXPECT_EQ(geometry.name(), geometry_names_[i]);

      EXPECT_TRUE(CompareMatrices(
          geometry_state_.GetPoseInFrame(geometry.id()).GetAsMatrix34(),
          X_FGs_[i].GetAsMatrix34()));
    }
  }
  EXPECT_EQ(static_cast<int>(gs_tester_.get_geometry_world_poses().size()),
            single_tree_rigid_geometry_count());
  EXPECT_EQ(gs_tester_.get_frame_parent_poses().size(), kFrameCount + 1);
}

// The reported geometry ids should be inclusive and ordered.
TEST_F(GeometryStateTest, GetAllGeometryIds) {
  // We want to make sure the list spans sources.
  const SourceId test_source = NewSource("all_ids");
  const GeometryId local_anchored_id = geometry_state_.RegisterAnchoredGeometry(
      test_source, make_unique<GeometryInstance>(
                       RigidTransformd(), make_unique<Sphere>(1), "anchored"));
  geometry_state_.AssignRole(test_source, local_anchored_id,
                             ProximityProperties{});
  SetUpSingleSourceTree();

  vector<GeometryId> expected_ids(geometries_);
  expected_ids.push_back(local_anchored_id);
  expected_ids.push_back(anchored_geometry_);
  std::sort(expected_ids.begin(), expected_ids.end());

  const vector<GeometryId> all_ids =
      geometry_state_.GetAllGeometryIds(/* role = */ std::nullopt);
  EXPECT_EQ(all_ids, expected_ids);

  expected_ids.clear();
  expected_ids.push_back(local_anchored_id);

  const vector<GeometryId> proximity_ids =
      geometry_state_.GetAllGeometryIds(Role::kProximity);
  EXPECT_EQ(proximity_ids, expected_ids);
}

// Confirms that a GeometrySet can be converted into a set of geometry ids.
TEST_F(GeometryStateTest, GetGeometryIds) {
  // Configure a scene where *every* geometry has a proximity role, but other
  // geometries selectively have illustration and/or perception roles.
  SetUpSingleSourceTree(Assign::kProximity);
  ASSERT_LE(6, single_tree_dynamic_rigid_geometry_count());

  // Frame 0 has proximity only. Frame 1 has proximity and perception.
  PerceptionProperties perception;
  perception.AddProperty("phong", "diffuse", Rgba{0.8, 0.8, 0.8, 1.0});
  perception.AddProperty("label", "id", RenderLabel::kDontCare);
  geometry_state_.AssignRole(source_id_, geometries_[2], perception);
  geometry_state_.AssignRole(source_id_, geometries_[3], perception);

  // Frame 2 has *one* geometry with illustration and proximity and one frame
  // with all three.
  IllustrationProperties illustration;
  illustration.AddProperty("phong", "diffuse", Rgba{0.8, 0.8, 0.8, 1.0});
  geometry_state_.AssignRole(source_id_, geometries_[4], illustration);
  geometry_state_.AssignRole(source_id_, geometries_[5], illustration);
  geometry_state_.AssignRole(source_id_, geometries_[5], perception);

  // Collect up all geometries (in various categories) for convenience.
  unordered_set<GeometryId> all_geometries(geometries_.begin(),
                                           geometries_.end());
  all_geometries.insert(anchored_geometry_);
  const unordered_set<GeometryId> perception_geometries{
      geometries_[2], geometries_[3], geometries_[5]};
  const unordered_set<GeometryId> illustration_geometries{geometries_[4],
                                                          geometries_[5]};

  {
    // Case: All geometries referenced solely by their parent frames.
    unordered_set<FrameId> all_frames(frames_.begin(), frames_.end());
    all_frames.insert(InternalFrame::world_frame_id());
    EXPECT_EQ(
        geometry_state_.GetGeometryIds(GeometrySet(all_frames), std::nullopt),
        all_geometries);
    EXPECT_EQ(geometry_state_.GetGeometryIds(GeometrySet(all_frames),
                                             Role::kProximity),
              all_geometries);
    EXPECT_EQ(geometry_state_.GetGeometryIds(GeometrySet(all_frames),
                                             Role::kPerception),
              perception_geometries);
    EXPECT_EQ(geometry_state_.GetGeometryIds(GeometrySet(all_frames),
                                             Role::kIllustration),
              illustration_geometries);
  }

  {
    // Case: All geometries referenced explicitly; this implicitly confirms
    // "culling" of otherwise explicitly declared GeometryIds.
    EXPECT_EQ(geometry_state_.GetGeometryIds(GeometrySet(all_geometries),
                                             std::nullopt),
              all_geometries);
    EXPECT_EQ(geometry_state_.GetGeometryIds(GeometrySet(all_geometries),
                                             Role::kProximity),
              all_geometries);
    EXPECT_EQ(geometry_state_.GetGeometryIds(GeometrySet(all_geometries),
                                             Role::kPerception),
              perception_geometries);
    EXPECT_EQ(geometry_state_.GetGeometryIds(GeometrySet(all_geometries),
                                             Role::kIllustration),
              illustration_geometries);
  }

  {
    // Case: Redundant specification; a geometry id is included explicitly as
    // well as its parent frame. If it *should* be included, it will appear
    // once. If it should *not* appear, it will not be included at all.
    EXPECT_EQ(geometry_state_.GetFrameId(geometries_[2]), frames_[1]);
    EXPECT_EQ(geometry_state_.GetFrameId(geometries_[3]), frames_[1]);
    const GeometrySet redundant_set({geometries_[2]}, {frames_[1]});
    EXPECT_EQ(geometry_state_.GetGeometryIds(redundant_set, std::nullopt),
              unordered_set<GeometryId>({geometries_[2], geometries_[3]}));
    EXPECT_EQ(geometry_state_.GetGeometryIds(redundant_set, Role::kProximity),
              unordered_set<GeometryId>({geometries_[2], geometries_[3]}));
    EXPECT_EQ(geometry_state_.GetGeometryIds(redundant_set, Role::kPerception),
              unordered_set<GeometryId>({geometries_[2], geometries_[3]}));
    EXPECT_EQ(
        geometry_state_.GetGeometryIds(redundant_set, Role::kIllustration),
        unordered_set<GeometryId>());
  }
}

// Tests the GetNum*Geometry*Methods.
TEST_F(GeometryStateTest, GetNumGeometryTests) {
  SetUpWithRigidAndDeformableGeometries(Assign::kProximity);
  EXPECT_EQ(total_geometry_count(), geometry_state_.get_num_geometries());
  EXPECT_EQ(total_geometry_count(),
            geometry_state_.NumGeometriesWithRole(Role::kProximity));
  EXPECT_EQ(0, geometry_state_.NumGeometriesWithRole(Role::kPerception));
  EXPECT_EQ(0, geometry_state_.NumGeometriesWithRole(Role::kIllustration));

  for (int i = 0; i < kFrameCount; ++i) {
    EXPECT_EQ(kGeometryCount,
              geometry_state_.NumGeometriesForFrame(frames_[i]));
    EXPECT_EQ(kGeometryCount, geometry_state_.NumGeometriesForFrameWithRole(
                                  frames_[i], Role::kProximity));
    EXPECT_EQ(0, geometry_state_.NumGeometriesForFrameWithRole(
                     frames_[i], Role::kPerception));
    EXPECT_EQ(0, geometry_state_.NumGeometriesForFrameWithRole(
                     frames_[i], Role::kIllustration));
  }

  EXPECT_EQ(1,
            geometry_state_.NumDeformableGeometriesWithRole(Role::kProximity));
  EXPECT_EQ(0,
            geometry_state_.NumDeformableGeometriesWithRole(Role::kPerception));
  EXPECT_EQ(
      0, geometry_state_.NumDeformableGeometriesWithRole(Role::kIllustration));
}

// Tests GetGeometries(FrameId, Role).
TEST_F(GeometryStateTest, GetGeometryTest) {
  SetUpSingleSourceTree(Assign::kProximity);
  // We want some heterogeneity in the roles the registered geometries have.
  //  - The single source tree applies proximity properties to all of the
  //    tree geometries.
  //  - We'll add illustration properties to one geometry in every frame.
  //  - We'll add an *additional* geometry to every frame (with no role).
  //  So, every geometry should report:
  //    - kGeometryCount + 1 = 3 total geometries.
  //    - 2 with proximity role
  //    - 1 with illustration role
  //    - 1 with unassigned
  //    - 0 with perception.

  // Assign illustration roles.
  for (int i = 0; i < kFrameCount * kGeometryCount; i += kGeometryCount) {
    const GeometryId g_id = geometries_[i];
    geometry_state_.AssignRole(source_id_, g_id, IllustrationProperties());
  }

  const vector<GeometryId> empty_ids;
  for (int i = 0; i < kFrameCount; ++i) {
    const FrameId f_id = frames_[i];

    // Add new geometry with unassigned role.
    auto instance = make_unique<GeometryInstance>(
        RigidTransformd::Identity(), make_unique<Sphere>(1), "shape");
    const GeometryId new_id =
        geometry_state_.RegisterGeometry(source_id_, f_id, std::move(instance));

    // Build the expected answers.
    const auto first_geometry_iter = geometries_.begin() + i * kGeometryCount;
    const vector<GeometryId> proximity_ids{
        first_geometry_iter, first_geometry_iter + kGeometryCount};
    vector<GeometryId> all_ids(proximity_ids);
    all_ids.push_back(new_id);

    EXPECT_EQ(geometry_state_.NumGeometriesForFrame(f_id), kGeometryCount + 1);
    EXPECT_EQ(geometry_state_.GetGeometries(f_id, Role::kPerception),
              empty_ids);
    EXPECT_EQ(geometry_state_.GetGeometries(f_id, Role::kProximity),
              proximity_ids);
    EXPECT_EQ(geometry_state_.GetGeometries(f_id, Role::kIllustration),
              vector<GeometryId>{geometries_[i * kGeometryCount]});
    EXPECT_EQ(geometry_state_.GetGeometries(f_id, Role::kUnassigned),
              vector<GeometryId>{new_id});
    EXPECT_EQ(geometry_state_.GetGeometries(f_id, std::nullopt), all_ids);
  }
}

// Tests that an attempt to add a frame to an invalid source throws an exception
// with meaningful message.
TEST_F(GeometryStateTest, AddFrameToInvalidSource) {
  const SourceId s_id = SourceId::get_new_id();  // Not a registered source.
  DRAKE_ASSERT_THROWS_MESSAGE(
      geometry_state_.RegisterFrame(s_id, *frame_),
      "Referenced geometry source \\d+ is not registered.");
}

// Tests that a duplicate-named frame added to a valid source throws an
// exception with a meaningful message.
TEST_F(GeometryStateTest, DuplicateFrameName) {
  const SourceId s_id = NewSource();
  geometry_state_.RegisterFrame(s_id, *frame_);
  DRAKE_ASSERT_THROWS_MESSAGE(
      geometry_state_.RegisterFrame(s_id, GeometryFrame(frame_->name())),
      ".*source 'default_source'.*duplicate name 'ref_frame'");
}

// Tests that adding another frame named "world" to the self source throws an
// exception.
TEST_F(GeometryStateTest, ManyWorldsRefuted) {
  DRAKE_ASSERT_THROWS_MESSAGE(
      geometry_state_.RegisterFrame(gs_tester_.get_self_source_id(),
                                    GeometryFrame("world")),
      ".*source 'SceneGraphInternal'.*duplicate name 'world'");
}

// Tests that a frame added to a valid source appears in the source's frames.
TEST_F(GeometryStateTest, AddFirstFrameToValidSource) {
  const SourceId s_id = NewSource();
  const FrameId fid = geometry_state_.RegisterFrame(s_id, *frame_);
  EXPECT_EQ(fid, frame_->id());
  EXPECT_TRUE(geometry_state_.BelongsToSource(fid, s_id));
  const auto& frame_set = geometry_state_.FramesForSource(s_id);
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
  const auto& frame_set = geometry_state_.FramesForSource(s_id);
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
    const auto& frame_set = geometry_state_.FramesForSource(new_s_id);
    EXPECT_NE(frame_set.find(fid), frame_set.end());
    EXPECT_EQ(frame_set.size(), 1);
  }
  // Confirm original source is unchanged.
  {
    const auto& frame_set = geometry_state_.FramesForSource(s_id);
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
      geometry_state_.RegisterFrame(s_id, *frame_),
      "Registering frame with an id that has already been registered: \\d+");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterFrame(s_id, f_id, *frame_),
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
      geometry_state_.RegisterGeometry(s_id, f_id, std::move(instance_));
  EXPECT_EQ(g_id, expected_g_id);
  EXPECT_EQ(geometry_state_.GetFrameId(g_id), f_id);
  EXPECT_TRUE(geometry_state_.BelongsToSource(g_id, s_id));
  const RigidTransformd& X_FG = geometry_state_.GetPoseInFrame(g_id);
  EXPECT_TRUE(
      CompareMatrices(X_FG.GetAsMatrix34(), instance_pose_.GetAsMatrix34()));

  EXPECT_TRUE(gs_tester_.get_frames().at(f_id).has_child(g_id));
  const auto& geometry = gs_tester_.get_geometries().at(g_id);
  EXPECT_TRUE(geometry.is_child_of_frame(f_id));
}

// Confirms that when adding a geometry G, X_WG is up-to-date with the best
// possible data.
TEST_F(GeometryStateTest, AddGeometryUpdatesX_WG) {
  // Configure the basic tree and set all frame poses to the default poses.
  const SourceId s_id = SetUpSingleSourceTree(Assign::kProximity);
  FramePoseVector<double> poses;
  for (int f = 0; f < static_cast<int>(frames_.size()); ++f) {
    poses.set_value(frames_[f], X_PFs_[f]);
  }
  gs_tester_.SetFramePoses(s_id, poses, &gs_tester_.mutable_kinematics_data());
  gs_tester_.FinalizePoseUpdate();

  // Registering a geometry to a frame F should report X_WG = X_WF * X_FG.

  vector<pair<FrameId, RigidTransformd>> parent_frames = {
      {InternalFrame::world_frame_id(), RigidTransformd{}},
      {frames_[0], X_WFs_[0]},
      {frames_[1], X_WFs_[1]},
      {frames_[2], X_WFs_[2]}};
  for (const auto& [f_id, X_WF] : parent_frames) {
    auto instance = make_unique<GeometryInstance>(
        instance_pose_, make_unique<Sphere>(1.0), "instance");
    const GeometryId g_id =
        geometry_state_.RegisterGeometry(s_id, f_id, std::move(instance));
    const RigidTransformd X_WG_expected = X_WF * instance_->pose();
    EXPECT_TRUE(
        CompareMatrices(geometry_state_.get_pose_in_world(g_id).GetAsMatrix34(),
                        X_WG_expected.GetAsMatrix34()));
  }
}

// Confirms that registering two geometries with the same id causes failure.
TEST_F(GeometryStateTest, RegisterDuplicateGeometry) {
  const SourceId s_id = NewSource();
  const FrameId f_id = geometry_state_.RegisterFrame(s_id, *frame_);
  auto instance_copy = make_unique<GeometryInstance>(*instance_);
  geometry_state_.RegisterGeometry(s_id, f_id, std::move(instance_));
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterGeometry(s_id, f_id, std::move(instance_copy)),
      "Registering geometry with an id that has already been registered: \\d+");
}

// Tests registration of geometry on invalid source.
TEST_F(GeometryStateTest, RegisterGeometryMissingSource) {
  const SourceId s_id = SourceId::get_new_id();
  const FrameId f_id = FrameId::get_new_id();
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterGeometry(s_id, f_id, std::move(instance_)),
      "Referenced geometry source \\d+ is not registered.");
}

// Tests registration of geometry on valid source and non-existent frame.
TEST_F(GeometryStateTest, RegisterGeometryMissingFrame) {
  const SourceId s_id = NewSource();
  const FrameId f_id = FrameId::get_new_id();
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterGeometry(s_id, f_id, std::move(instance_)),
      "Referenced frame \\d+ for source \\d+\\,"
      " but the frame doesn't belong to the source.");
}

// Tests error resulting from passing a null GeometryInstance.
TEST_F(GeometryStateTest, RegisterNullGeometry) {
  const SourceId s_id = NewSource();
  const FrameId f_id = geometry_state_.RegisterFrame(s_id, *frame_);
  unique_ptr<GeometryInstance> null_geometry;
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterGeometry(s_id, f_id, std::move(null_geometry)),
      "Registering null geometry to frame \\d+, on source \\d+.");
}

// Tests the registration of anchored geometry.
TEST_F(GeometryStateTest, RegisterAnchoredGeometry) {
  const SourceId s_id = NewSource("new source");
  auto instance = make_unique<GeometryInstance>(
      RigidTransformd::Identity(), make_unique<Sphere>(1), "sphere");
  const GeometryId expected_g_id = instance->id();
  const auto g_id =
      geometry_state_.RegisterAnchoredGeometry(s_id, std::move(instance));
  EXPECT_EQ(g_id, expected_g_id);
  EXPECT_TRUE(geometry_state_.BelongsToSource(g_id, s_id));
  const InternalGeometry* g = gs_tester_.GetGeometry(g_id);
  EXPECT_NE(g, nullptr);
  EXPECT_EQ(InternalFrame::world_frame_id(), g->frame_id());
  EXPECT_EQ(s_id, g->source_id());
}

// Confirms that registering two geometries with the same id causes failure.
TEST_F(GeometryStateTest, RegisterDuplicateAnchoredGeometry) {
  const SourceId s_id = NewSource();
  auto instance_copy = make_unique<GeometryInstance>(*instance_);
  geometry_state_.RegisterAnchoredGeometry(s_id, std::move(instance_));
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterAnchoredGeometry(s_id, std::move(instance_copy)),
      "Registering geometry with an id that has already been "
      "registered: \\d+");
}

// Tests the attempt to register anchored geometry on an invalid source.
TEST_F(GeometryStateTest, RegisterAnchoredGeometryInvalidSource) {
  auto instance = make_unique<GeometryInstance>(
      RigidTransformd::Identity(), make_unique<Sphere>(1), "sphere");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterAnchoredGeometry(SourceId::get_new_id(),
                                               std::move(instance)),
      "Referenced geometry source \\d+ is not registered.");
}

// Tests the response of attempting to register a null pointer GeometryInstance
// as anchored geometry.
TEST_F(GeometryStateTest, RegisterAnchoredNullGeometry) {
  unique_ptr<GeometryInstance> instance;
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterAnchoredGeometry(SourceId::get_new_id(),
                                               std::move(instance)),
      "Registering null geometry to frame \\d+, on source \\d+.");
}

// Tests registering a deformable geometry.
TEST_F(GeometryStateTest, RegisterDeformableGeometry) {
  const SourceId s_id = NewSource("new source");
  const FrameId f_id = geometry_state_.RegisterFrame(s_id, *frame_);
  const Sphere sphere(1.0);
  constexpr double kRezHint = 0.5;
  const VolumeMesh<double> expected_mesh =
      internal::MakeSphereVolumeMesh<double>(
          sphere, kRezHint,
          internal::TessellationStrategy::kDenseInteriorVertices);

  /* Adding a deformable geometry to non-world frame throws. */
  auto instance1 = make_unique<GeometryInstance>(
      RigidTransformd::Identity(), make_unique<Sphere>(sphere), "sphere");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterDeformableGeometry(
          s_id, f_id, std::move(instance1), kRezHint),
      "Registering deformable geometry.*non-world frame.*");

  /* Successful registration of deformable geometry. */
  const RigidTransformd X_WG(AngleAxis<double>(M_PI_2, Vector3d::UnitX()),
                             Vector3d{1, 2, 3});
  auto instance2 = make_unique<GeometryInstance>(
      X_WG, make_unique<Sphere>(sphere), "sphere");
  const GeometryId expected_g_id = instance2->id();
  const auto g_id = geometry_state_.RegisterDeformableGeometry(
      s_id, InternalFrame::world_frame_id(), std::move(instance2), kRezHint);
  EXPECT_EQ(g_id, expected_g_id);
  EXPECT_TRUE(geometry_state_.IsDeformableGeometry(g_id));

  /* Registers a non-deformable geometry. */
  auto instance3 = make_unique<GeometryInstance>(
      RigidTransformd::Identity(), make_unique<Sphere>(sphere), "sphere");
  const GeometryId non_deformable_g_id =
      geometry_state_.RegisterGeometry(s_id, f_id, std::move(instance3));
  EXPECT_FALSE(geometry_state_.IsDeformableGeometry(non_deformable_g_id));

  // Verify the reference mesh of the deformable geometry matches the input.
  const VolumeMesh<double>* reference_mesh =
      geometry_state_.GetReferenceMesh(g_id);
  ASSERT_NE(reference_mesh, nullptr);
  EXPECT_TRUE(reference_mesh->Equal(expected_mesh));

  // Querying the _reference_ pose is fine.
  EXPECT_TRUE(geometry_state_.GetPoseInFrame(g_id).IsExactlyEqualTo(X_WG));

  // Querying _current_ pose on deformable geometry throws. (Deformable
  // geometries are characterized by vertex positions via
  // get_configurations_in_world.)
  EXPECT_THROW(geometry_state_.get_pose_in_world(g_id), std::exception);

  // Verify vertex positions can be retrieved.
  // TODO(xuchenhan-tri): Strengthen this test when we can set vertex positions.
  const VectorX<double>& q_WG =
      geometry_state_.get_configurations_in_world(g_id);
  EXPECT_EQ(q_WG.size(), expected_mesh.num_vertices() * 3);

  // Verify that deformable geometries are dynamic (One deformable and one
  // dynamic non-deformable).
  EXPECT_EQ(geometry_state_.NumDynamicGeometries(), 2);
  EXPECT_EQ(geometry_state_.NumDeformableGeometries(), 1);

  // Verifies that GetAllDeformableGeometryIds() collect all deformable
  // geometry ids and _none_ of the non-deformable geometry ids.
  const std::vector<GeometryId> deformable_ids =
      geometry_state_.GetAllDeformableGeometryIds();
  ASSERT_EQ(deformable_ids.size(), 1);
  EXPECT_EQ(deformable_ids[0], g_id);
}

// GetAllDeformableGeometryIds should pull *only* deformable geometry ids and
// should get them from all sources. Finally, the list should be ordered.
TEST_F(GeometryStateTest, GetAllDeformableGeometryIds) {
  /* Populate the world with some non-deformable geometries to ignore. */
  SetUpSingleSourceTree();
  const SourceId s1_id = NewSource("source1");
  const SourceId s2_id = NewSource("source2");
  const Sphere sphere(1.0);
  constexpr double kRezHint = 0.5;

  /* Add one deformable geometry per source. */
  auto instance1 = make_unique<GeometryInstance>(RigidTransformd::Identity(),
                                                 sphere.Clone(), "deformable1");
  auto instance2 = make_unique<GeometryInstance>(RigidTransformd::Identity(),
                                                 sphere.Clone(), "deformable2");

  /* Our expected outcome. */
  vector<GeometryId> expected_ids{instance1->id(), instance2->id()};
  std::sort(expected_ids.begin(), expected_ids.end());

  /* Register in counter-intuitive order. */
  geometry_state_.RegisterDeformableGeometry(
      s2_id, InternalFrame::world_frame_id(), std::move(instance2), kRezHint);
  geometry_state_.RegisterDeformableGeometry(
      s1_id, InternalFrame::world_frame_id(), std::move(instance1), kRezHint);

  /* Confirm that we get the expected ids in the expected order. */
  const vector<GeometryId> deformable_ids =
      geometry_state_.GetAllDeformableGeometryIds();
  ASSERT_EQ(deformable_ids, expected_ids);
}

/* This test covers the data maintenance *within* GeometryState. It confirms
 that shape and pose change, but all other InternalGeometry fields remain the
 same.

 There are per-role tests below that provide coverage for the unchanged
 role properties and confirm that the shape has changed in the various engines.

 This test makes a simplifying assumption: multiple roles don't interfere in
 the process. So, we can test a single geometry with all three roles and don't
 have to test geometries with all possible role configurations. */
TEST_F(GeometryStateTest, ChangeShapeInternals) {
  const SourceId s_id = SetUpSingleSourceTree(
      Assign::kProximity | Assign::kPerception | Assign::kIllustration);

  const GeometryId g_id = geometries_[0];
  const InternalGeometry* geometry = gs_tester_.GetGeometry(g_id);
  const InternalGeometry original_geo(*geometry);
  const RigidTransformd X_GG2(Vector3d(1, 2, 3));
  const RigidTransformd X_FG2 = X_GG2 * geometry->X_FG();

  // Changing from sphere to box.
  ASSERT_EQ(geometry->shape().type_name(), Sphere(1.0).type_name());
  const Box new_shape(1.0, 2.0, 3.0);

  geometry_state_.ChangeShape(s_id, g_id, new_shape, X_FG2);

  // We've modified it in place; the address hasn't changed.
  ASSERT_EQ(geometry, gs_tester_.GetGeometry(g_id));

  // Shape and pose have changed.
  EXPECT_EQ(geometry->shape().type_name(), Box(1, 1, 1).type_name());
  EXPECT_TRUE(
      CompareMatrices(X_FG2.GetAsMatrix34(), geometry->X_FG().GetAsMatrix34()));

  // Nothing else has.
  EXPECT_EQ(original_geo.id(), geometry->id());
  EXPECT_EQ(original_geo.name(), geometry->name());
  EXPECT_EQ(original_geo.source_id(), geometry->source_id());
  EXPECT_EQ(original_geo.frame_id(), geometry->frame_id());
  EXPECT_EQ(original_geo.is_deformable(), geometry->is_deformable());
  EXPECT_EQ(original_geo.is_dynamic(), geometry->is_dynamic());
}

/* Changes a shape with the illustration role. The only observable indications
 of a successful change is that the shape and pose have changed *and* that the
 illustration version has changed. */
TEST_F(GeometryStateTest, ChangeShapeIllustration) {
  const SourceId s_id = SetUpSingleSourceTree(Assign::kIllustration);

  const GeometryId g_id = geometries_[0];
  const InternalGeometry* geometry = gs_tester_.GetGeometry(g_id);
  const InternalGeometry original_geo(*geometry);

  // Changing from sphere to box.
  ASSERT_EQ(geometry->shape().type_name(), Sphere(1.0).type_name());
  const Box new_shape(1.0, 2.0, 3.0);

  const GeometryVersion pre_version = geometry_state_.geometry_version();
  geometry_state_.ChangeShape(s_id, g_id, new_shape, geometry->X_FG());
  const GeometryVersion post_version = geometry_state_.geometry_version();

  // Only illustration version has changed.
  EXPECT_FALSE(post_version.IsSameAs(pre_version, Role::kIllustration));
  EXPECT_TRUE(post_version.IsSameAs(pre_version, Role::kPerception));
  EXPECT_TRUE(post_version.IsSameAs(pre_version, Role::kProximity));

  // IllustrationProperties haven't changed. In this test, every property gets
  // this arbitrary (test, value) property; we'll confirm that it's still there
  // as *evidence* that the properties haven't changed.
  EXPECT_EQ(
      original_geo.illustration_properties()->GetProperty<int>("test", "value"),
      geometry->illustration_properties()->GetProperty<int>("test", "value"));
}

// Engine to test the ChangeShape() API. Change shape registers a new shape,
// and passes a new pose. This render engine records the last geometry id
// registered and removed. We'll confirm that both happen in response to a call
// to ChangeShape().
class ChangeShapeRenderEngine : public DummyRenderEngine {
 public:
  ChangeShapeRenderEngine() : DummyRenderEngine() {}

  optional<GeometryId> get_and_clear_last_registered_id() {
    optional<GeometryId> last_value = registered_id_;
    registered_id_ = std::nullopt;
    return last_value;
  }

  optional<GeometryId> get_and_clear_last_removed_id() {
    optional<GeometryId> last_value = removed_id_;
    removed_id_ = std::nullopt;
    return last_value;
  }

 protected:
  bool DoRegisterVisual(GeometryId id, const Shape&,
                        const PerceptionProperties&,
                        const math::RigidTransformd&) override {
    registered_id_ = id;
    return true;
  }

  bool DoRemoveGeometry(GeometryId id) override {
    removed_id_ = id;
    return true;
  }

 private:
  optional<GeometryId> registered_id_{};
  optional<GeometryId> removed_id_{};
};

ChangeShapeRenderEngine* AddRenderer(const char* name,
                                     GeometryState<double>* state) {
  state->AddRenderer(name, make_unique<ChangeShapeRenderEngine>());
  auto* engine =
      const_cast<render::RenderEngine*>(state->GetRenderEngineByName(name));
  auto* test_engine = dynamic_cast<ChangeShapeRenderEngine*>(engine);
  DRAKE_DEMAND(test_engine != nullptr);
  return test_engine;
}

/* Changes a shape with the perception role. In this case, we'll confirm that
 the shape in the render engine has changed by using a custom render engine
 implementation that will report that its APIs have been called. */
TEST_F(GeometryStateTest, ChangeShapePerception) {
  // We'll add to render engines to make sure that all render engines get
  // processed.
  ChangeShapeRenderEngine* renderer1 =
      AddRenderer("renderer1", &geometry_state_);
  ChangeShapeRenderEngine* renderer2 =
      AddRenderer("renderer2", &geometry_state_);

  const SourceId s_id = SetUpSingleSourceTree(Assign::kPerception);

  const GeometryId g_id = geometries_[0];
  const InternalGeometry* geometry = gs_tester_.GetGeometry(g_id);
  const InternalGeometry original_geo(*geometry);

  // Confirm pre-test conditions; neither reports g_id as last registered or
  // removed.
  ASSERT_NE(renderer1->get_and_clear_last_registered_id(), g_id);
  ASSERT_NE(renderer2->get_and_clear_last_registered_id(), g_id);
  ASSERT_NE(renderer1->get_and_clear_last_removed_id(), g_id);
  ASSERT_NE(renderer2->get_and_clear_last_removed_id(), g_id);

  ASSERT_EQ(geometry->shape().type_name(), Sphere(1.0).type_name());
  const Box new_shape(0.1, 0.2, 0.3);

  const GeometryVersion pre_version = geometry_state_.geometry_version();
  geometry_state_.ChangeShape(s_id, g_id, new_shape, geometry->X_FG());
  const GeometryVersion post_version = geometry_state_.geometry_version();

  // Only illustration version has changed.
  EXPECT_TRUE(post_version.IsSameAs(pre_version, Role::kIllustration));
  EXPECT_FALSE(post_version.IsSameAs(pre_version, Role::kPerception));
  EXPECT_TRUE(post_version.IsSameAs(pre_version, Role::kProximity));

  // PerceptionProperties haven't changed. In this test, every property gets
  // this arbitrary (test, value) property; we'll confirm that it's still
  // there as *evidence* that the properties haven't changed.
  EXPECT_EQ(
      original_geo.perception_properties()->GetProperty<int>("test", "value"),
      geometry->perception_properties()->GetProperty<int>("test", "value"));

  // Each render engine now reports that the changed geometry id was both
  // removed and registered (presumably in the correct order).
  ASSERT_EQ(renderer1->get_and_clear_last_registered_id(), g_id);
  ASSERT_EQ(renderer2->get_and_clear_last_registered_id(), g_id);
  ASSERT_EQ(renderer1->get_and_clear_last_removed_id(), g_id);
  ASSERT_EQ(renderer2->get_and_clear_last_removed_id(), g_id);
}

/* Changes a shape with the proximity role. In this case, we'll confirm that the
 shape in the proximity engine has changed by performing a query before and
 after, showing that the query result changes as expected. */
TEST_F(GeometryStateTest, ChangeShapeProximity) {
  const SourceId s_id = SetUpSingleSourceTree(Assign::kProximity);

  const GeometryId g_id = geometries_[0];
  const InternalGeometry* geometry = gs_tester_.GetGeometry(g_id);
  const InternalGeometry original_geo(*geometry);

  // Record the old distance.
  const SignedDistancePair<double> old_distance =
      geometry_state_.ComputeSignedDistancePairClosestPoints(g_id,
                                                             geometries_[2]);

  // Changing from sphere to a *small* box (should be smaller than the sphere).
  // The distance between the two shapes should get *larger*.
  ASSERT_EQ(geometry->shape().type_name(), Sphere(1.0).type_name());
  const Box new_shape(0.1, 0.2, 0.3);

  const GeometryVersion pre_version = geometry_state_.geometry_version();
  geometry_state_.ChangeShape(s_id, g_id, new_shape, geometry->X_FG());
  const GeometryVersion post_version = geometry_state_.geometry_version();

  // Only proximity version has changed.
  EXPECT_TRUE(post_version.IsSameAs(pre_version, Role::kIllustration));
  EXPECT_TRUE(post_version.IsSameAs(pre_version, Role::kPerception));
  EXPECT_FALSE(post_version.IsSameAs(pre_version, Role::kProximity));

  // ProximityProperties haven't changed. In this test, every property gets
  // this arbitrary (test, value) property; we'll confirm that it's still there
  // as *evidence* that the properties haven't changed.
  EXPECT_EQ(
      original_geo.proximity_properties()->GetProperty<int>("test", "value"),
      geometry->proximity_properties()->GetProperty<int>("test", "value"));

  // Now confirm an increase in distance.
  const SignedDistancePair<double> new_distance =
      geometry_state_.ComputeSignedDistancePairClosestPoints(g_id,
                                                             geometries_[2]);
  EXPECT_GT(new_distance.distance, old_distance.distance);
}

/* This tests for two things:
 1. SetGeometryConfiguration() successfully sets the geometry configuration for
    a given geometry, and
 2. FinalizeConfigurationUpdate propagates the configuration to the
    ProximityEngine.
 We test for 2 by initializing a rigid geometry in contact with the deformable
 geometry and verifying that the rigid/deformable geometry pair are no longer in
 contact after the deformable vertices are moved away by
 SetGeometryConfiguration() *and* FinalizeConfigurationUpdate() is invoked. */
TEST_F(GeometryStateTest, SetGeometryConfiguration) {
  const SourceId s_id = NewSource("new source");
  // Add a deformable geometry with proximity property.
  auto deformable_instance = make_unique<GeometryInstance>(
      RigidTransformd::Identity(), make_unique<Sphere>(1.0),
      "deformable sphere");
  deformable_instance->set_proximity_properties(ProximityProperties());
  const auto deformable_id = geometry_state_.RegisterDeformableGeometry(
      s_id, InternalFrame::world_frame_id(), std::move(deformable_instance),
      /* resolution_hint */ 2.0);
  // Add a rigid geometry with a resolution hint (so that it can be in contact
  // with deformable geometries).
  auto rigid_instance = make_unique<GeometryInstance>(
      RigidTransformd(Vector3d(1, 0, 0)), make_unique<Box>(1.5, 1.5, 1.5),
      "rigid_box");
  ProximityProperties rigid_properties;
  rigid_properties.AddProperty(internal::kHydroGroup, internal::kRezHint, 1.0);
  rigid_properties.AddProperty(internal::kHydroGroup, internal::kElastic, 1e6);
  rigid_instance->set_proximity_properties(std::move(rigid_properties));
  const FrameId f_id =
      geometry_state_.RegisterFrame(s_id, GeometryFrame("frame"));
  geometry_state_.RegisterGeometry(s_id, f_id, std::move(rigid_instance));

  // Update the deformable geometry's configuration by translating it away from
  // the rigid geometry.
  const VectorX<double> q_WG_default =
      geometry_state_.get_configurations_in_world(deformable_id);
  VectorX<double> q_WG = q_WG_default;
  VectorX<double> offset_W = 10.0 * VectorX<double>::Ones(q_WG.size());
  q_WG += offset_W;
  EXPECT_FALSE(CompareMatrices(q_WG_default, q_WG, 0.1));
  GeometryConfigurationVector<double> configurations;
  configurations.set_value(deformable_id, q_WG);
  gs_tester_.SetGeometryConfiguration(s_id, configurations,
                                      &gs_tester_.mutable_kinematics_data());
  EXPECT_EQ(q_WG, geometry_state_.get_configurations_in_world(deformable_id));

  // Verify that the two geometries are still in contact *before"
  // FinalizeConfigurationUpdate() is called.
  internal::DeformableContact<double> contacts;
  geometry_state_.ComputeDeformableContact(&contacts);
  ASSERT_EQ(contacts.contact_surfaces().size(), 1);

  // Verify that the two geometries are no longer in contact *after"
  // FinalizeConfigurationUpdate() is called.
  gs_tester_.FinalizeConfigurationUpdate();
  geometry_state_.ComputeDeformableContact(&contacts);
  ASSERT_EQ(contacts.contact_surfaces().size(), 0);
}

// Tests GetDrivenMeshConfigurationsInWorld and GetDrivenRenderMeshes.
// We register the coarsest possible sphere mesh for a deformable geometry, use
// the fact that the mesh is an octahedron, and confirm the driven meshes are
// correctly registered and retrieved.
TEST_F(GeometryStateTest, DrivenMeshes) {
  const SourceId s_id = NewSource("new source");
  auto deformable_instance = make_unique<GeometryInstance>(
      RigidTransformd::Identity(), make_unique<Sphere>(1.0),
      "deformable sphere");
  deformable_instance->set_proximity_properties(ProximityProperties());
  deformable_instance->set_perception_properties(PerceptionProperties());
  // Specify the diffuse color for illustration properties only. This diffuse
  // color should be captured by the material of the corresponding driven
  // RenderMesh. The RenderMeshes for the other roles will not have a material.
  IllustrationProperties illustration_properties;
  illustration_properties.AddProperty("phong", "diffuse",
                                      Rgba(0.1, 0.2, 0.3, 0.4));
  deformable_instance->set_illustration_properties(illustration_properties);
  // Make sure the resolution hint is large enough so we end up with an
  // octahedron.
  const auto deformable_id = geometry_state_.RegisterDeformableGeometry(
      s_id, InternalFrame::world_frame_id(), std::move(deformable_instance),
      /* resolution_hint */ 2.0);

  // Confirm that the driven mesh with each role is the surface of the
  // octahedron.
  for (const auto role : std::vector<Role>{
           Role::kIllustration, Role::kPerception, Role::kProximity}) {
    const std::vector<internal::RenderMesh>& render_meshes =
        geometry_state_.GetDrivenRenderMeshes(deformable_id, role);
    ASSERT_EQ(ssize(render_meshes), 1);
    const auto& mesh = render_meshes[0];
    // The octahedron has 6 vertices and 8 faces.
    EXPECT_EQ(mesh.positions.rows(), 6);
    EXPECT_EQ(mesh.indices.rows(), 8);
    if (role == Role::kIllustration) {
      // The illustration mesh should have the illustration properties.
      EXPECT_EQ(mesh.material->diffuse, Rgba(0.1, 0.2, 0.3, 0.4));
    } else {
      EXPECT_FALSE(mesh.material.has_value());
    }
  }

  const std::vector<VectorX<double>> reference_proximity_configuration =
      geometry_state_.GetDrivenMeshConfigurationsInWorld(deformable_id,
                                                         Role::kProximity);
  ASSERT_EQ(reference_proximity_configuration.size(), 1);
  for (const auto role : std::vector<Role>{
           Role::kIllustration, Role::kPerception, Role::kProximity}) {
    const std::vector<VectorX<double>> configuration =
        geometry_state_.GetDrivenMeshConfigurationsInWorld(deformable_id, role);
    EXPECT_EQ(configuration, reference_proximity_configuration);
  }
  // Scale the geometry by a factor of 2.0 and confirm that the driven meshes
  // are also scaled.
  const VectorX<double> reference_q_WG =
      geometry_state_.get_configurations_in_world(deformable_id);
  GeometryConfigurationVector<double> new_configurations;
  new_configurations.set_value(deformable_id, 2.0 * reference_q_WG);
  gs_tester_.SetGeometryConfiguration(s_id, new_configurations,
                                      &gs_tester_.mutable_kinematics_data());
  for (const auto role : std::vector<Role>{
           Role::kIllustration, Role::kPerception, Role::kProximity}) {
    const std::vector<VectorX<double>> configuration =
        geometry_state_.GetDrivenMeshConfigurationsInWorld(deformable_id, role);
    EXPECT_TRUE(CompareMatrices(configuration[0],
                                reference_proximity_configuration[0]));
  }

  // Trying to get driven meshes for a non-existent role should throw.
  geometry_state_.RemoveRole(s_id, deformable_id, Role::kIllustration);
  EXPECT_THROW(
      geometry_state_.GetDrivenRenderMeshes(deformable_id, Role::kIllustration),
      std::exception);
  EXPECT_THROW(geometry_state_.GetDrivenMeshConfigurationsInWorld(
                   deformable_id, Role::kIllustration),
               std::exception);
}

// Tests the RemoveGeometry() functionality. This action will have several
// effects which will all be confirmed.
//  - The geometry is no longer known to the proximity engine.
//  - The geometry is no longer known to the render engine.
//  - confirm that various operation still work post remove:
//    - pose updates
//    - registration of geometry
//    - assignment of roles.
TEST_F(GeometryStateTest, RemoveGeometry) {
  // Every geometry gets proximity and perception properties.
  const SourceId s_id =
      SetUpSingleSourceTree(Assign::kProximity | Assign::kPerception);

  // Pose all of the frames to the default poses (X_PFs_).
  FramePoseVector<double> poses;
  for (int f = 0; f < static_cast<int>(frames_.size()); ++f) {
    poses.set_value(frames_[f], X_PFs_[f]);
  }
  gs_tester_.SetFramePoses(s_id, poses, &gs_tester_.mutable_kinematics_data());
  gs_tester_.FinalizePoseUpdate();

  // The geometry to remove and  its parent frame.
  const GeometryId g_id = geometries_[0];
  const FrameId f_id = frames_[0];

  // Confirm initial state.
  ASSERT_EQ(geometry_state_.GetFrameId(g_id), f_id);
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            single_tree_rigid_geometry_count());
  EXPECT_EQ(geometry_state_.NumDynamicGeometries(),
            single_tree_dynamic_rigid_geometry_count());
  // We assume that role assignment works (tested below), such that the
  // geometries are registered with the appropriate engines.

  geometry_state_.RemoveGeometry(s_id, g_id);

  EXPECT_EQ(geometry_state_.get_num_geometries(),
            single_tree_rigid_geometry_count() - 1);
  EXPECT_EQ(geometry_state_.NumDynamicGeometries(),
            single_tree_dynamic_rigid_geometry_count() - 1);

  EXPECT_FALSE(gs_tester_.get_frames().at(f_id).has_child(g_id));
  EXPECT_FALSE(gs_tester_.get_geometries().contains(g_id));

  // Confirm that, post removal, updating poses still works.
  DRAKE_EXPECT_NO_THROW(gs_tester_.FinalizePoseUpdate());

  // Adding a new geometry; various operations should still "work" (in the sense
  // that nothing explodes).
  GeometryId added_id;
  DRAKE_EXPECT_NO_THROW(
      added_id = geometry_state_.RegisterGeometry(
          source_id_, frames_[0],
          make_unique<GeometryInstance>(RigidTransformd::Identity(),
                                        make_unique<Sphere>(1), "newest")));

  // Adding proximity role to the new geometry brings the total number of
  // dynamic geometries with proximity roles back up to the original value.
  DRAKE_EXPECT_NO_THROW(
      geometry_state_.AssignRole(source_id_, added_id, ProximityProperties()));

  // Now remove the *final* geometry, it should still keep things valid -- in
  // other words, we should still be able to finalized poses.
  DRAKE_EXPECT_NO_THROW(geometry_state_.RemoveGeometry(s_id, added_id));
  // Confirm that, post removal, updating poses still works.
  DRAKE_EXPECT_NO_THROW(gs_tester_.FinalizePoseUpdate());
}

// Tests the response to invalid use of RemoveGeometry.
TEST_F(GeometryStateTest, RemoveGeometryInvalid) {
  const SourceId s_id = SetUpSingleSourceTree();

  // Case: Invalid source id, valid geometry id.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveGeometry(SourceId::get_new_id(), geometries_[0]),
      "Referenced geometry source \\d+ is not registered.");

  // Case: Invalid geometry id, valid source id.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveGeometry(s_id, GeometryId::get_new_id()),
      "Referenced geometry \\d+ has not been registered.");

  // Case: Valid geometry and source, but geometry belongs to different source.
  const SourceId s_id2 = geometry_state_.RegisterNewSource("new_source");
  const FrameId frame_id = geometry_state_.RegisterFrame(s_id2, *frame_);
  EXPECT_EQ(geometry_state_.get_num_frames(), single_tree_frame_count() + 1);
  const GeometryId g_id = geometry_state_.RegisterGeometry(
      s_id2, frame_id,
      make_unique<GeometryInstance>(RigidTransformd::Identity(),
                                    unique_ptr<Shape>(new Sphere(1)), "new"));
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            single_tree_rigid_geometry_count() + 1);
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveGeometry(s_id, g_id),
      "Trying to remove geometry \\d+ from source \\d+.+geometry doesn't "
      "belong.+");
}

// Tests removal of anchored geometry.
TEST_F(GeometryStateTest, RemoveAnchoredGeometry) {
  const SourceId s_id = SetUpSingleSourceTree(Assign::kProximity);

  const Vector3<double> normal_W{0, 1, 0};
  const Vector3<double> p_WB{1, 1, 1};
  const auto anchored_id_1 = geometry_state_.RegisterAnchoredGeometry(
      s_id,
      make_unique<GeometryInstance>(HalfSpace::MakePose(normal_W, p_WB),
                                    make_unique<HalfSpace>(), "anchored1"));
  geometry_state_.AssignRole(s_id, anchored_id_1, ProximityProperties());
  // Confirm conditions of having added the anchored geometry.
  EXPECT_TRUE(geometry_state_.BelongsToSource(anchored_id_1, s_id));

  geometry_state_.RemoveGeometry(s_id, anchored_geometry_);

  EXPECT_EQ(gs_tester_.GetGeometry(anchored_geometry_), nullptr);
}

// Tests removal of geometry when collision filters are present. As geometries
// are removed, surrounding filter semantics should *not* change.
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

  // Note: removing anchored geometry because it was the last registered.
  geometry_state_.RemoveGeometry(s_id, anchored_geometry_);
  // Now remove some interior geometry -- make sure filter semantics don't get
  // crossed.
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
      "Referenced geometry source \\d+ is not registered.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.BelongsToSource(GeometryId::get_new_id(), source_id),
      "Referenced geometry source \\d+ is not registered.");
  SetUpSingleSourceTree();
  const GeometryId anchored_id = geometry_state_.RegisterAnchoredGeometry(
      source_id_,
      make_unique<GeometryInstance>(RigidTransformd::Identity(),
                                    make_unique<Sphere>(1), "sphere"));
  // Valid frame/geometry ids.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.BelongsToSource(frames_[0], source_id),
      "Referenced geometry source \\d+ is not registered.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.BelongsToSource(geometries_[0], source_id),
      "Referenced geometry source \\d+ is not registered.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.BelongsToSource(anchored_id, source_id),
      "Referenced geometry source \\d+ is not registered.");
}

// This tests the source ownership functionality for frames - a function which
// reports if a frame belongs to the specified source.
TEST_F(GeometryStateTest, SourceOwnershipFrameId) {
  const SourceId s_id = SetUpSingleSourceTree();
  // Test for invalid frame.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.BelongsToSource(FrameId::get_new_id(), s_id),
      "Referenced frame \\d+ has not been registered.");
  // Test for valid frame.
  EXPECT_TRUE(geometry_state_.BelongsToSource(frames_[0], s_id));
}

// This tests the source ownership functionality for geometry - a function which
// reports if a geometry belongs to the specified source. It examines dynamic
// and anchored geometry.
TEST_F(GeometryStateTest, SourceOwnershipGeometryId) {
  const SourceId s_id = SetUpSingleSourceTree();
  const GeometryId anchored_id = geometry_state_.RegisterAnchoredGeometry(
      s_id, make_unique<GeometryInstance>(RigidTransformd::Identity(),
                                          make_unique<Sphere>(1), "sphere"));
  // Test for invalid geometry.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.BelongsToSource(GeometryId::get_new_id(), s_id),
      "Referenced geometry \\d+ has not been registered.");
  // Test for valid geometry.
  EXPECT_TRUE(geometry_state_.BelongsToSource(geometries_[0], s_id));
  EXPECT_TRUE(geometry_state_.BelongsToSource(anchored_id, s_id));
}

// This confirms the failure state of calling GeometryState::GetFrameId with a
// bad geometry identifier.
TEST_F(GeometryStateTest, GetFrameIdFromBadId) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.GetFrameId(GeometryId::get_new_id()),
      "Referenced geometry \\d+ has not been registered.");
}

// Tests the validation of the ids provided in a frame kinematics vector.
TEST_F(GeometryStateTest, ValidateFrameIds) {
  const SourceId s_id = SetUpSingleSourceTree();
  FramePoseVector<double> frame_set;
  for (auto frame_id : frames_) {
    frame_set.set_value(frame_id, RigidTransformd::Identity());
  }
  // Case: frame ids are valid.
  DRAKE_EXPECT_NO_THROW(gs_tester_.ValidateFrameIds(s_id, frame_set));

  // Case: Right number, wrong frames.
  FramePoseVector<double> frame_set_2;
  for (int i = 0; i < kFrameCount; ++i) {
    frame_set_2.set_value(FrameId::get_new_id(), RigidTransformd::Identity());
  }
  DRAKE_EXPECT_THROWS_MESSAGE(
      gs_tester_.ValidateFrameIds(s_id, frame_set_2),
      "Registered frame id \\(\\d+\\) belonging to source \\d+ was not found "
      "in the provided kinematics data.");

  // Case: Too few frames.
  FramePoseVector<double> frame_set_3;
  for (int i = 0; i < kFrameCount - 1; ++i) {
    frame_set.set_value(frames_[i], RigidTransformd::Identity());
  }
  DRAKE_EXPECT_THROWS_MESSAGE(
      gs_tester_.ValidateFrameIds(s_id, frame_set_3),
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
  vector<RigidTransformd> frame_poses;
  for (int i = 0; i < kFrameCount; ++i) {
    frame_poses.push_back(RigidTransformd::Identity());
  }

  auto make_pose_vector = [&frame_poses, this]() -> FramePoseVector<double> {
    const int count = static_cast<int>(this->frames_.size());
    FramePoseVector<double> poses;
    for (int i = 0; i < count; ++i) {
      poses.set_value(this->frames_[i], frame_poses[i]);
    }
    return poses;
  };

  // NOTE: Don't re-order the tests; they rely on an accumulative set of changes
  // to the `frame_poses` vector of poses.
  const int total_geom = single_tree_dynamic_rigid_geometry_count();

  // Case 1: Set all frames to identity poses. The world pose of all the
  // geometry should be that of the geometry in its frame.
  FramePoseVector<double> poses1 = make_pose_vector();
  gs_tester_.SetFramePoses(s_id, poses1, &gs_tester_.mutable_kinematics_data());
  const auto& world_poses = gs_tester_.get_geometry_world_poses();
  for (int i = 0; i < total_geom; ++i) {
    const GeometryId id = geometries_[i];
    EXPECT_TRUE(CompareMatrices(world_poses.at(id).GetAsMatrix34(),
                                X_FGs_[i].GetAsMatrix34()));
  }

  // Case 2: Move the two *root* frames 1 unit in the +y direction. The f2 will
  // stay at the identity.
  // The final geometry poses should all be offset by 1 unit in the y.
  const RigidTransformd offset{Translation3d{0, 1, 0}};
  frame_poses[0] = offset;
  frame_poses[1] = offset;
  FramePoseVector<double> poses2 = make_pose_vector();
  gs_tester_.SetFramePoses(s_id, poses2, &gs_tester_.mutable_kinematics_data());
  for (int i = 0; i < total_geom; ++i) {
    const GeometryId id = geometries_[i];
    EXPECT_TRUE(CompareMatrices(world_poses.at(id).GetAsMatrix34(),
                                (offset * X_FGs_[i]).GetAsMatrix34()));
  }

  // Case 3: All frames get set to move up one unit. This will leave geometries
  // 0, 1, 2, & 3 moved up 1, and geometries 4 & 5 moved up two.
  frame_poses[2] = offset;
  FramePoseVector<double> poses3 = make_pose_vector();
  gs_tester_.SetFramePoses(s_id, poses3, &gs_tester_.mutable_kinematics_data());
  for (int i = 0; i < total_geom; ++i) {
    const GeometryId id = geometries_[i];
    if (i < (kFrameCount - 1) * kGeometryCount) {
      EXPECT_TRUE(CompareMatrices(world_poses.at(id).GetAsMatrix34(),
                                  (offset * X_FGs_[i]).GetAsMatrix34()));
    } else {
      EXPECT_TRUE(
          CompareMatrices(world_poses.at(id).GetAsMatrix34(),
                          (offset * offset * X_FGs_[i]).GetAsMatrix34()));
    }
  }
}

// Test various frame property queries.
TEST_F(GeometryStateTest, QueryFrameProperties) {
  const SourceId s_id = SetUpSingleSourceTree();
  const FrameId world = InternalFrame::world_frame_id();

  // Query frame group.
  EXPECT_EQ(geometry_state_.GetFrameGroup(frames_[0]), 0);
  EXPECT_EQ(geometry_state_.GetFrameGroup(world),
            InternalFrame::world_frame_group());
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.GetFrameGroup(FrameId::get_new_id()),
      "No frame group available for invalid frame id: \\d+");

  // Query frame name.
  EXPECT_EQ(geometry_state_.GetName(frames_[0]), "f0");
  EXPECT_EQ(geometry_state_.GetName(world), "world");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.GetName(FrameId::get_new_id()),
      "No frame name available for invalid frame id: \\d+");

  // Set the frame poses to query geometry and frame poses.
  FramePoseVector<double> poses;
  for (int i = 0; i < kFrameCount; ++i) poses.set_value(frames_[i], X_PFs_[i]);
  gs_tester_.SetFramePoses(s_id, poses, &gs_tester_.mutable_kinematics_data());

  EXPECT_TRUE(CompareMatrices(
      geometry_state_.get_pose_in_world(frames_[0]).GetAsMatrix34(),
      X_WFs_[0].GetAsMatrix34()));
  EXPECT_TRUE(
      CompareMatrices(geometry_state_.get_pose_in_world(world).GetAsMatrix34(),
                      RigidTransformd::Identity().GetAsMatrix34()));
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.get_pose_in_world(FrameId::get_new_id()),
      "No world pose available for invalid frame id: \\d+");

  // This assumes that geometry parent belongs to frame 0.
  const RigidTransformd X_WG = X_WFs_[0] * X_FGs_[0];
  EXPECT_TRUE(CompareMatrices(
      geometry_state_.get_pose_in_world(geometries_[0]).GetAsMatrix34(),
      X_WG.GetAsMatrix34()));
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.get_pose_in_world(GeometryId::get_new_id()),
      "No world pose available for invalid geometry id: \\d+");
  EXPECT_TRUE(CompareMatrices(
      geometry_state_.get_pose_in_world(anchored_geometry_).GetAsMatrix34(),
      X_WA_.GetAsMatrix34()));

  EXPECT_TRUE(CompareMatrices(
      geometry_state_.get_pose_in_parent(frames_[0]).GetAsMatrix34(),
      X_PFs_[0].GetAsMatrix34()));
  EXPECT_TRUE(
      CompareMatrices(geometry_state_.get_pose_in_parent(world).GetAsMatrix34(),
                      RigidTransformd::Identity().GetAsMatrix34()));
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.get_pose_in_parent(FrameId::get_new_id()),
      "No pose available for invalid frame id: \\d+");
}

TEST_F(GeometryStateTest, TestCollisionCandidates) {
  SetUpSingleSourceTree(Assign::kProximity);

  // The explicit enumeration of candidate pairs. geometries 0 & 1, 2 & 3, and
  // 4 & 5 are siblings, therefore they are not valid candidate pairs. All other
  // combinations _are_.
  set<pair<GeometryId, GeometryId>> expected_candidates = {
      {geometries_[0], geometries_[2]},
      {geometries_[0], geometries_[3]},
      {geometries_[0], geometries_[4]},
      {geometries_[0], geometries_[5]},
      {geometries_[0], anchored_geometry_},
      {geometries_[1], geometries_[2]},
      {geometries_[1], geometries_[3]},
      {geometries_[1], geometries_[4]},
      {geometries_[1], geometries_[5]},
      {geometries_[1], anchored_geometry_},
      {geometries_[2], geometries_[4]},
      {geometries_[2], geometries_[5]},
      {geometries_[2], anchored_geometry_},
      {geometries_[3], geometries_[4]},
      {geometries_[3], geometries_[5]},
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
                result << fmt::format(" ({}, {})", p.first, p.second);
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

  // This relies on the correctness of CollisionFilterManager(). It implicitly
  // tests that GeometryState::collision_filter_manager() produces a valid
  // manager.
  while (!expected_candidates.empty()) {
    const auto pair = expected_candidates.begin();
    geometry_state_.collision_filter_manager().Apply(
        CollisionFilterDeclaration().ExcludeBetween(GeometrySet{pair->first},
                                                    GeometrySet{pair->second}));
    expected_candidates.erase(pair);
    EXPECT_TRUE(candidates_in_set(geometry_state_.GetCollisionCandidates(),
                                  expected_candidates));
  }
  // We've filtered everything, should report as empty.
  EXPECT_TRUE(candidates_in_set(geometry_state_.GetCollisionCandidates(),
                                expected_candidates));
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
  gs_tester_.SetFramePoses(source_id_, poses,
                           &gs_tester_.mutable_kinematics_data());
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
  const RigidTransformd pose{Translation3d{5.5, 0, 0}};
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
  geometry_state_.collision_filter_manager().Apply(
      CollisionFilterDeclaration().ExcludeBetween(
          GeometrySet{added_id}, GeometrySet{anchored_geometry_}));
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
  geometry_state_.collision_filter_manager().Apply(
      CollisionFilterDeclaration().ExcludeBetween(
          GeometrySet{added_id}, GeometrySet{anchored_geometry_}));
  pairs = geometry_state_.ComputePointPairPenetration();
  EXPECT_EQ(static_cast<int>(pairs.size()), expected_collisions);
}

// Tests two aspects of GeometryState collision filter behavior:
//
//   1. No collision filters are applied to a deformable geometry (it and the
//      anchored geometry are both registered to the world frame) automatically.
//   2. GeometryState::CollectIds() respects the scope parameter in applying
//      collision filters.
//
// This is tested by calling ComputeDeformableContact(). In its initial state,
// we should report contact between a deformable geometry and *all* rigid
// geometries. Then applying a collision filter between the deformable geometry
// and one of the rigid geometries will reduce the number of contacts.
TEST_F(GeometryStateTest, CollisionFilterRespectsScope) {
  const SourceId s_id = NewSource("new source");
  auto rigid_instance0 = make_unique<GeometryInstance>(
      RigidTransformd(Vector3d(0.75, 0, 0)), make_unique<Box>(1.0, 1.0, 1.0),
      "rigid_box0");
  auto rigid_instance1 = make_unique<GeometryInstance>(
      RigidTransformd(Vector3d(-0.75, 0, 0)), make_unique<Box>(1.0, 1.0, 1.0),
      "rigid_box1");

  // Give all rigid geometries resolution hint and a hydroelastic modulus so
  // that they can collide with deformable geometries.
  ProximityProperties rigid_properties;
  rigid_properties.AddProperty(internal::kHydroGroup, internal::kRezHint, 10.0);
  rigid_properties.AddProperty(internal::kHydroGroup, internal::kElastic, 1e6);
  rigid_instance0->set_proximity_properties(rigid_properties);
  rigid_instance1->set_proximity_properties(rigid_properties);

  // Register two rigid geometries, one anchored and one dynamic.
  const FrameId f_id =
      geometry_state_.RegisterFrame(s_id, GeometryFrame("frame"));
  GeometryId rigid_id0 =
      geometry_state_.RegisterGeometry(s_id, f_id, std::move(rigid_instance0));
  GeometryId rigid_id1 = geometry_state_.RegisterGeometry(
      s_id, InternalFrame::world_frame_id(), std::move(rigid_instance1));

  // Register a deformable geometry that's guaranteed to be in collision
  // with both rigid geometries.
  const Sphere sphere(1.0);
  constexpr double kRezHint = 2.0;
  auto instance = make_unique<GeometryInstance>(
      RigidTransformd::Identity(), make_unique<Sphere>(sphere), "deformable");
  instance->set_proximity_properties(ProximityProperties());
  GeometryId deformable_id = geometry_state_.RegisterDeformableGeometry(
      s_id, InternalFrame::world_frame_id(), std::move(instance), kRezHint);
  internal::DeformableContact<double> contacts;
  geometry_state_.ComputeDeformableContact(&contacts);
  EXPECT_EQ(contacts.contact_surfaces().size(), 2);

  // Attempting to filter collisions with scope omitting deformable geometries
  // should have no effect on the number of collisions.
  geometry_state_.collision_filter_manager().Apply(
      CollisionFilterDeclaration(CollisionFilterScope::kOmitDeformable)
          .ExcludeBetween(GeometrySet{deformable_id},
                          GeometrySet(geometries_)));
  geometry_state_.ComputeDeformableContact(&contacts);
  EXPECT_EQ(contacts.contact_surfaces().size(), 2);

  // Filter with the kAll flag as the scope should have an effect. The collision
  // between the deformable geometry and the dynamic rigid geometry is filtered,
  // so the number of collisions should now be 1.
  geometry_state_.collision_filter_manager().Apply(
      CollisionFilterDeclaration(CollisionFilterScope::kAll)
          .ExcludeBetween(GeometrySet{deformable_id}, GeometrySet{rigid_id0}));
  geometry_state_.ComputeDeformableContact(&contacts);
  EXPECT_EQ(contacts.contact_surfaces().size(), 1);

  // Now filter the collision between the deformable geometry and the anchored
  // rigid geometry. The number of collisions should now be 0.
  geometry_state_.collision_filter_manager().Apply(
      CollisionFilterDeclaration(CollisionFilterScope::kAll)
          .ExcludeBetween(GeometrySet{deformable_id}, GeometrySet{rigid_id1}));
  geometry_state_.ComputeDeformableContact(&contacts);
  EXPECT_EQ(contacts.contact_surfaces().size(), 0);
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
      ".* " + to_string(geometries_[1]) + " does not have a proximity role");

  // Case: Second argument does not have a proximity role.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.CollisionFiltered(geometries_[0], geometries_[1]),
      ".* " + to_string(geometries_[1]) + " does not have a proximity role");

  // Case: Neither argument has a proximity role.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.CollisionFiltered(geometries_[1], geometries_[2]),
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
      "Can't report collision filter status between geometries .* " +
          to_string(bad_id) + " is not a valid geometry");

  // Case: Second argument is bad.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.CollisionFiltered(geometries_[0], bad_id),
      "Can't report collision filter status between geometries .* " +
          to_string(bad_id) + " is not a valid geometry");

  // Case: Both arguments are bad.
  DRAKE_EXPECT_THROWS_MESSAGE(geometry_state_.CollisionFiltered(bad_id, bad_id),
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
      EXPECT_EQ(geometry_state_.GetGeometryIdByName(
                    frames_[f], Role::kProximity, geometry_names_[g_index]),
                expected_id);
      // Look up with non-canonical name.
      EXPECT_EQ(
          geometry_state_.GetGeometryIdByName(frames_[f], Role::kProximity,
                                              " " + geometry_names_[g_index]),
          expected_id);
    }
  }

  // Grab anchored.
  EXPECT_EQ(
      geometry_state_.GetGeometryIdByName(InternalFrame::world_frame_id(),
                                          Role::kProximity, anchored_name_),
      anchored_geometry_);

  // Failure cases.

  // Look up with the wrong name.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.GetGeometryIdByName(frames_[0], Role::kProximity, "bad"),
      "The frame 'f0' .\\d+. has no geometry with the role 'proximity' and the "
      "canonical name 'bad'. The names associated with this frame/role are "
      "\\{\\d+_g1, \\d+_g0\\}.");

  // Bad frame id.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.GetGeometryIdByName(FrameId::get_new_id(),
                                          Role::kUnassigned, "irrelevant"),
      "Referenced frame \\d+ has not been registered.");

  // Bad *anchored* geometry name.
  const FrameId world_id = gs_tester_.get_world_frame();
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.GetGeometryIdByName(world_id, Role::kUnassigned, "bad"),
      "The frame 'world' .\\d+. has no geometry with the role 'unassigned' "
      "and the canonical name '.+'. The names associated with this frame/role "
      "are \\{\\}.");

  // Bad *dynamic* geometry name.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.GetGeometryIdByName(frames_[0], Role::kUnassigned,
                                          "bad_name"),
      "The frame '.+?' .\\d+. has no geometry with the role 'unassigned' and "
      "the canonical name '.+'. The names associated with this frame/role are "
      "\\{\\}.");

  // Multiple unassigned geometries with the same name.

  const string dummy_name("duplicate");
  for (int i = 0; i < 2; ++i) {
    geometry_state_.RegisterGeometry(
        source_id_, frames_[0],
        make_unique<GeometryInstance>(RigidTransformd::Identity(),
                                      make_unique<Sphere>(1), dummy_name));
  }
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.GetGeometryIdByName(frames_[0], Role::kUnassigned,
                                          dummy_name),
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
        make_unique<GeometryInstance>(RigidTransformd::Identity(),
                                      make_unique<Sphere>(1), " " + name));
    EXPECT_EQ(geometry_state_.GetName(id), name);
  }

  // Valid name that is unchanged after trimming is stored as is.
  // Note: This assigns a geometry fo the *same* name to a *different* frame.
  {
    const GeometryId id = geometry_state_.RegisterGeometry(
        source_id_, frames_[1],
        make_unique<GeometryInstance>(RigidTransformd::Identity(),
                                      make_unique<Sphere>(1), name));
    EXPECT_EQ(geometry_state_.GetName(id), name);
  }
}

// Confirms parental relationships between frames are stored
TEST_F(GeometryStateTest, GeometryAncestryStorage) {
  SetUpSingleSourceTree();

  // {child, parent}
  std::map<std::string, std::string> expected_relationships = {
      {"world", "world"},
      {"f0", "world"},
      {"f1", "world"},
      {"f2", "f1"},
  };

  ASSERT_EQ(expected_relationships.size(), geometry_state_.get_num_frames());

  for (const FrameId frame_id : geometry_state_.get_frame_ids()) {
    const std::string& frame_name = geometry_state_.GetName(frame_id);
    const FrameId parent_frame_id = geometry_state_.GetParentFrame(frame_id);
    const std::string& parent_frame_name =
        geometry_state_.GetName(parent_frame_id);
    EXPECT_EQ(parent_frame_name, expected_relationships[frame_name]);
  }
}

// Tests the logic for confirming if a name is valid or not.
TEST_F(GeometryStateTest, GeometryNameValidation) {
  SetUpSingleSourceTree(Assign::kProximity);

  // Case: Invalid frame should throw (regardless of name contents or role).
  DRAKE_EXPECT_THROWS_MESSAGE(geometry_state_.IsValidGeometryName(
                                  FrameId::get_new_id(), Role::kProximity, ""),
                              "Given frame id is not valid: \\d+");

  auto expect_bad_name = [this](const string& name,
                                const string& exception_message,
                                const string& printable_name) {
    EXPECT_FALSE(
        geometry_state_.IsValidGeometryName(frames_[0], Role::kProximity, name))
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
  for (const char* const s : {"\n", " \n\t", " \f", "\v", "\r", "\ntest"}) {
    EXPECT_TRUE(
        geometry_state_.IsValidGeometryName(frames_[0], Role::kProximity, s));
  }
}

// Simply tests that when a role is assigned to a geometry, that it successfully
// reports that it has that role.
TEST_F(GeometryStateTest, AssignRolesToGeometry) {
  SetUpSingleSourceTree();

  // We need at least 8 geometries to run through all role permutations. Add
  // geometries until we're there.
  const RigidTransformd pose = RigidTransformd::Identity();
  for (int i = 0; i < 8 - single_tree_dynamic_rigid_geometry_count(); ++i) {
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
    if (has_perception != geometry->has_perception_role()) {
      failure << "Perception role: "
              << (has_perception ? "expected, but not found"
                                 : "not expected, but found");
      passes = false;
    }
    if (has_illustration != (geometry->illustration_properties() != nullptr)) {
      failure << "Illustration role: "
              << (has_illustration ? "expected, but not found"
                                   : "not expected, but found. ");
      passes = false;
    }
    if (passes) {
      return ::testing::AssertionSuccess();
    } else {
      return failure;
    }
  };

  // Given three role types, assign all eight types of assignments.
  for (int i = 0; i < 8; ++i) {
    const bool proximity = i & 0x1;
    const bool illustration = i & 0x2;
    const bool perception = i & 0x4;
    const GeometryId id = geometries_[i];
    EXPECT_TRUE(has_expected_roles(id, false, false, false)) << fmt::format(
        "Geometry {} at index ({}) didn't start without roles", id, i);
    set_roles(id, proximity, perception, illustration);
    EXPECT_TRUE(has_expected_roles(id, proximity, perception, illustration))
        << fmt::format("Incorrect roles for geometry {} at index ({}).", id, i);
  }

  // Confirm it works on anchored geometry. Pick, arbitrarily, assigning
  // proximity and illustration roles.
  EXPECT_TRUE(has_expected_roles(anchored_geometry_, false, false, false));
  set_roles(anchored_geometry_, true, false, true);
  EXPECT_TRUE(has_expected_roles(anchored_geometry_, true, false, true));
}

// Test the APIs/functionality related to the computation of a convex hull.
// Currently, one is generated only for geometries of certain types (see
// ProximityEngine::NeedsConvexHull()) with the proximity role. This tests the
// logic for generating, removing, and reporting convex hulls.
TEST_F(GeometryStateTest, ConvexHullForProximityGeometry) {
  SetUpSingleSourceTree(Assign::kProximity);

  // A shape that doesn't require a convex hull won't have one. All geometries
  // added by SetUpSingleSourceTree() are spheres and get no convex hulls.
  DRAKE_DEMAND(geometry_state_.GetProximityProperties(geometries_[0]) !=
               nullptr);
  EXPECT_EQ(geometry_state_.GetConvexHull(geometries_[0]), nullptr);

  // A shape that *does* require a convex hull has one.
  const std::string mesh_name =
      FindResourceOrThrow("drake/geometry/test/cube_with_hole.obj");
  const GeometryId mesh_id = geometry_state_.RegisterAnchoredGeometry(
      source_id_, make_unique<GeometryInstance>(
                      RigidTransformd{}, Mesh(mesh_name, 1), "mesh_with_hull"));
  geometry_state_.AssignRole(source_id_, mesh_id, ProximityProperties());
  EXPECT_NE(geometry_state_.GetConvexHull(mesh_id), nullptr);

  // Updating properties leaves the convex hull defined.
  EXPECT_FALSE(geometry_state_.GetProximityProperties(mesh_id)->HasProperty(
      "test", "value"));
  ProximityProperties alt_props;
  alt_props.AddProperty("test", "value", 17);
  geometry_state_.AssignRole(source_id_, mesh_id, alt_props,
                             RoleAssign::kReplace);
  DRAKE_DEMAND(geometry_state_.GetProximityProperties(mesh_id)->HasProperty(
      "test", "value"));
  EXPECT_NE(geometry_state_.GetConvexHull(mesh_id), nullptr);

  // Changing the shape clears the convex hull.
  geometry_state_.ChangeShape(source_id_, mesh_id, Sphere(2), std::nullopt);
  EXPECT_EQ(geometry_state_.GetConvexHull(mesh_id), nullptr);
}

// The implementation of computing the OBB is tested in calc_obb_test.cc. This
// test simply confirms that the right function is called.
TEST_F(GeometryStateTest, GetObbInGeometryFrame) {
  SetUpSingleSourceTree(Assign::kIllustration);
  const std::optional<Obb>& maybe_obb =
      geometry_state_.GetObbInGeometryFrame(geometries_[0]);
  EXPECT_TRUE(maybe_obb.has_value());
  EXPECT_EQ(maybe_obb->center(), Vector3d::Zero());
  EXPECT_TRUE(
      CompareMatrices(maybe_obb->half_width(), Vector3d(1, 1, 1), 1e-13));
}

// Test the ability to reassign proximity properties to a geometry that already
// has the proximity role.
TEST_F(GeometryStateTest, ModifyProximityProperties) {
  SetUpSingleSourceTree();
  // No roles assigned.
  EXPECT_EQ(gs_tester_.proximity_engine().num_geometries(), 0);

  ProximityProperties empty_props;
  ProximityProperties props1;
  props1.AddProperty("prop1", "value", 1);
  ProximityProperties props2;
  props2.AddProperty("prop2", "value", 2);
  AddRigidHydroelasticProperties(1.0, &props2);

  // Case: Can't reassign when it hasn't been assigned.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.AssignRole(source_id_, geometries_[0], empty_props,
                                 RoleAssign::kReplace),
      "Trying to replace the properties on geometry id \\d+ for the 'proximity'"
      " role.*");

  // Case: Try reassigning the properties.
  // We need to confirm that GeometryState gives ProximityEngine the chance to
  // update its representation based on the proximity properties. Currently,
  // the only way to do that is by looking at the hydroelastic representation.
  // Originally, it should have none, after the update, it should.
  const auto& hydroelastic_geometries =
      internal::ProximityEngineTester::hydroelastic_geometries(
          gs_tester_.proximity_engine());

  // Configure and confirm initial state.
  geometry_state_.AssignRole(source_id_, geometries_[0], props1);
  // One geometry now has the role.
  EXPECT_EQ(gs_tester_.proximity_engine().num_geometries(), 1);
  const ProximityProperties* props =
      geometry_state_.GetProximityProperties(geometries_[0]);
  EXPECT_NE(props, nullptr);
  EXPECT_TRUE(props->HasGroup("prop1"));
  EXPECT_TRUE(props->HasProperty("prop1", "value"));
  EXPECT_EQ(props->GetProperty<int>("prop1", "value"),
            props1.GetProperty<int>("prop1", "value"));
  EXPECT_EQ(hydroelastic_geometries.hydroelastic_type(geometries_[0]),
            internal::HydroelasticType::kUndefined);

  DRAKE_EXPECT_NO_THROW(geometry_state_.AssignRole(
      source_id_, geometries_[0], props2, RoleAssign::kReplace));

  // Confirm modification doesn't introduce duplicates; should still be one.
  EXPECT_EQ(gs_tester_.proximity_engine().num_geometries(), 1);

  props = geometry_state_.GetProximityProperties(geometries_[0]);
  EXPECT_NE(props, nullptr);
  EXPECT_TRUE(props->HasGroup("prop2"));
  EXPECT_FALSE(props->HasGroup("prop1"));
  EXPECT_TRUE(props->HasProperty("prop2", "value"));
  EXPECT_EQ(props->GetProperty<int>("prop2", "value"),
            props2.GetProperty<int>("prop2", "value"));
  EXPECT_EQ(hydroelastic_geometries.hydroelastic_type(geometries_[0]),
            internal::HydroelasticType::kRigid);
}

// Test the ability to reassign illustration properties to a geometry that
// already has the illustration role.
TEST_F(GeometryStateTest, ModifyIllustrationProperties) {
  SetUpSingleSourceTree();

  // The geometry we're going to play with. Confirm that it starts without the
  // illustration role.
  const GeometryId geometry_id = geometries_[1];
  ASSERT_EQ(geometry_state_.GetIllustrationProperties(geometry_id), nullptr);

  IllustrationProperties empty_props;
  IllustrationProperties props1;
  props1.AddProperty("group1", "value", 1);
  IllustrationProperties props2;
  props2.AddProperty("group2", "value", 2);

  // Generally, we assume that illustration property replacement exercises the
  // same infrastructure as proximity, so we're going to omit some of the
  // error condition tests (e.g., replacing with nothing to replace).
  EXPECT_NO_THROW(
      geometry_state_.AssignRole(source_id_, geometry_id, empty_props));
  const IllustrationProperties* props =
      geometry_state_.GetIllustrationProperties(geometry_id);
  ASSERT_NE(props, nullptr);
  ASSERT_EQ(props->num_groups(), 1);  // Just the default group.

  // Now re-assign.
  EXPECT_NO_THROW(geometry_state_.AssignRole(source_id_, geometry_id, props1,
                                             RoleAssign::kReplace));
  props = geometry_state_.GetIllustrationProperties(geometry_id);
  ASSERT_NE(props, nullptr);
  ASSERT_TRUE(props->HasProperty("group1", "value"));
  ASSERT_FALSE(props->HasProperty("group2", "value"));

  // Now re-assign again.
  EXPECT_NO_THROW(geometry_state_.AssignRole(source_id_, geometry_id, props2,
                                             RoleAssign::kReplace));
  props = geometry_state_.GetIllustrationProperties(geometry_id);
  ASSERT_NE(props, nullptr);
  ASSERT_FALSE(props->HasProperty("group1", "value"));
  ASSERT_TRUE(props->HasProperty("group2", "value"));
}

// Test that attempting to reassign perception properties to a geometry that
// already has the perception role fails.
TEST_F(GeometryStateTest, ModifyPerceptionProperties) {
  SetUpSingleSourceTree();

  // Case: confirm throw for perception properties.
  PerceptionProperties perception_props =
      render_engine_->accepting_properties();
  perception_props.AddProperty("label", "id", RenderLabel(10));
  EXPECT_NO_THROW(
      geometry_state_.AssignRole(source_id_, geometries_[1], perception_props));
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.AssignRole(source_id_, geometries_[1], perception_props,
                                 RoleAssign::kReplace),
      "AssignRole\\(\\) with RoleAssign::kReplace does not work for perception "
      "properties");
}

// Tests the various Get*Properties(GeometryId) methods.
TEST_F(GeometryStateTest, RoleLookUp) {
  SetUpSingleSourceTree(Assign::kProximity | Assign::kIllustration |
                        Assign::kPerception);
  GeometryId no_role_id = geometry_state_.RegisterGeometry(
      source_id_, frames_[0],
      make_unique<GeometryInstance>(RigidTransformd::Identity(),
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
    return make_unique<GeometryInstance>(instance_pose_,
                                         make_unique<Sphere>(1.0), name);
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
        geometry_state_.RegisterGeometry(s_id, f_id, std::move(instance));

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
        geometry_state_.RegisterGeometry(s_id, f_id, std::move(instance));

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
        geometry_state_.RegisterGeometry(s_id, f_id, std::move(instance));

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
        geometry_state_.RegisterGeometry(s_id, f_id, std::move(instance));

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
      "Referenced geometry source \\d+ is not registered.");

  // Invalid geometry id.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.AssignRole(source_id_, GeometryId::get_new_id(),
                                 ProximityProperties()),
      "Referenced geometry \\d+ has not been registered.");

  // Geometry not owned by source.
  const SourceId other_source = geometry_state_.RegisterNewSource("alt_source");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.AssignRole(other_source, geometries_[0],
                                 ProximityProperties()),
      "Given geometry id \\d+ does not belong to the given source .*");

  // Redefinition of role - test each role individually to make sure it has
  // the right error message.
  DRAKE_EXPECT_NO_THROW(geometry_state_.AssignRole(source_id_, geometries_[0],
                                                   ProximityProperties()));
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.AssignRole(source_id_, geometries_[0],
                                 ProximityProperties()),
      "Trying to assign the 'proximity' role to geometry id \\d+ for the first "
      "time.*");

  DRAKE_EXPECT_NO_THROW(
      geometry_state_.AssignRole(source_id_, geometries_[0], perception_props));
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.AssignRole(source_id_, geometries_[0], perception_props),
      "Trying to assign the 'perception' role to geometry id \\d+ for the "
      "first time.*");

  DRAKE_EXPECT_NO_THROW(geometry_state_.AssignRole(source_id_, geometries_[0],
                                                   IllustrationProperties()));
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.AssignRole(source_id_, geometries_[0],
                                 IllustrationProperties()),
      "Trying to assign the 'illustration' role to geometry id \\d+ for the "
      "first time.*");

  // Addition of geometry with duplicate name -- no problem. Assigning it a
  // duplicate role -- bad.
  const GeometryId new_id = geometry_state_.RegisterGeometry(
      source_id_, frames_[0],
      make_unique<GeometryInstance>(RigidTransformd::Identity(),
                                    make_unique<Sphere>(1),
                                    geometry_names_[0]));
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.AssignRole(source_id_, new_id, ProximityProperties()),
      "The name .* has already been used by a geometry with the 'proximity' "
      "role.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.AssignRole(source_id_, new_id, IllustrationProperties()),
      "The name .* has already been used by a geometry with the 'illustration' "
      "role.");

  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.AssignRole(source_id_, new_id, perception_props),
      "The name .* has already been used by a geometry with the 'perception' "
      "role.");
}

// Tests the functionality that counts the number of children geometry a frame
// has for each role.
TEST_F(GeometryStateTest, ChildGeometryRoleCount) {
  // Defaults to *no* roles being set.
  SetUpSingleSourceTree();

  auto expected_roles =
      [this](FrameId f_id, int num_proximity, int num_perception,
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
                << fmt::to_string(role) << " role. Found " << actual_count;
      }
    }
    if (success) {
      return ::testing::AssertionSuccess();
    } else {
      return failure;
    }
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

// Confirms that attempting to remove the "unassigned" role has no effect.
TEST_F(GeometryStateTest, RemoveUnassignedRole) {
  SetUpSingleSourceTree(Assign::kProximity | Assign::kIllustration |
                        Assign::kPerception);

  EXPECT_EQ(
      geometry_state_.RemoveRole(source_id_, geometries_[0], Role::kUnassigned),
      0);
  EXPECT_EQ(
      geometry_state_.RemoveRole(source_id_, frames_[0], Role::kUnassigned), 0);

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
  auto temp_engine = make_unique<DummyRenderEngine>();
  const DummyRenderEngine& other_engine = *temp_engine;
  geometry_state_.AddRenderer(other_renderer_name, std::move(temp_engine));

  SetUpWithRigidAndDeformableGeometries(Assign::kPerception);

  // Note: we simulate the derived engine reporting removal of geometry ids via
  // set_geometry_remove({true | false}). When we expect the id won't be removed
  // (because it doesn't exist), we set it to false. Otherwise true.

  // Each geometry must report that it is in the default render engine, in
  // addition,
  //   a) report itself in the "other" render engine, xor
  //   b) be present in the `removed_from_renderer` set.
  auto confirm_renderers =
      [this, &other_engine](set<GeometryId> removed_from_renderer) {
        set<GeometryId> ids(geometries_.begin(), geometries_.end());
        ids.insert(anchored_geometry_);
        for (GeometryId id : ids) {
          // All should report in the dummy renderer.
          EXPECT_TRUE(render_engine_->has_geometry(id));
          // Should report in the renderer if it is *not* in the removed set.
          EXPECT_EQ(other_engine.has_geometry(id),
                    !removed_from_renderer.contains(id));
        }
      };
  set<GeometryId> removed_ids;

  // Confirm geometries assigned to _both_ renderers.
  confirm_renderers(removed_ids);

  // Case: Remove geometry from a single renderer (geometry should still be in
  // the default renderer).
  const GeometryId remove_id = geometries_[0];
  EXPECT_EQ(geometry_state_.RemoveFromRenderer(other_renderer_name, source_id_,
                                               remove_id),
            1);
  removed_ids.insert(remove_id);
  confirm_renderers(removed_ids);

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
      "Referenced geometry source .* is not registered.");

  // Case: GeometryId does not map to a registered geometry.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveFromRenderer(other_renderer_name, source_id_,
                                         GeometryId::get_new_id()),
      "Referenced geometry .* has not been registered.");

  // Case: GeometryId does not belong to SourceId.
  const SourceId source_id_2 =
      geometry_state_.RegisterNewSource("second_source");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveFromRenderer(other_renderer_name, source_id_2,
                                         remove_id),
      "Trying to remove geometry \\d+ from the renderer '.+', but the geometry "
      "doesn't belong to given source .+");
}

// The frame-centric version of the RemoveGeometryFromRenderer test.
TEST_F(GeometryStateTest, RemoveFrameFromRenderer) {
  // TODO(SeanCurtis-TRI): Consider refactoring this set-up code between _this_
  // test and the RemoveGeometryFromRenderer test.
  const string other_renderer_name = "alt_renderer";
  auto temp_engine = make_unique<DummyRenderEngine>();
  const DummyRenderEngine& other_engine = *temp_engine;
  geometry_state_.AddRenderer(other_renderer_name, std::move(temp_engine));

  SetUpSingleSourceTree(Assign::kPerception);

  // Note: we simulate the derived engine reporting removal of geometry ids via
  // set_geometry_remove({true | false}). When we expect the id won't be removed
  // (because it doesn't exist), we set it to false. Otherwise true.

  // Each geometry must report that it is in the default render engine, in
  // addition,
  //   a) report itself in the "other" render engine, xor
  //   b) be present in the `removed_from_renderer` set.
  auto confirm_renderers =
      [this, &other_engine](set<GeometryId> removed_from_renderer) {
        set<GeometryId> ids(geometries_.begin(), geometries_.end());
        ids.insert(anchored_geometry_);
        for (GeometryId id : ids) {
          // All should report in the dummy renderer.
          EXPECT_TRUE(render_engine_->has_geometry(id));
          // Should report in the renderer if it is *not* in the removed set.
          EXPECT_EQ(other_engine.has_geometry(id),
                    !removed_from_renderer.contains(id));
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
      "Referenced geometry source .* is not registered.");

  // Case: Frame does not map to a registered frame.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveFromRenderer(other_renderer_name, source_id_,
                                         FrameId::get_new_id()),
      "Referenced frame .* but the frame doesn't belong to the source.");

  // Case: FrameId does not belong to SourceId.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveFromRenderer(other_renderer_name, source_id_2,
                                         frames_[0]),
      "Referenced frame .+ but the frame doesn't belong to the source.");
}

TEST_F(GeometryStateTest, AddRendererAfterGeometry) {
  SetUpWithRigidAndDeformableGeometries(Assign::kPerception);
  // Add one geometry that has no perception properties.
  const GeometryId id_no_perception = geometry_state_.RegisterGeometry(
      source_id_, frames_[0],
      make_unique<GeometryInstance>(RigidTransformd::Identity(),
                                    make_unique<Sphere>(0.5), "shape"));
  // All geometries have perception role assigned and thus should be registered
  // with the accepting render engine.
  EXPECT_EQ(render_engine_->num_registered(), total_geometry_count());

  auto new_renderer = make_unique<DummyRenderEngine>();
  DummyRenderEngine* other_renderer = new_renderer.get();
  // The new renderer has no geometry assigned.
  EXPECT_EQ(other_renderer->num_registered(), 0);
  const string other_name = "other";
  geometry_state_.AddRenderer(other_name, std::move(new_renderer));
  // The new renderer also has all geometries registered.
  EXPECT_EQ(other_renderer->num_registered(), total_geometry_count());

  EXPECT_FALSE(other_renderer->has_geometry(id_no_perception));

  for (const GeometryId& id : geometries_) {
    EXPECT_TRUE(other_renderer->has_geometry(id));
  }
}

// Successful invocations of AddRenderer are implicit in SetupSingleSource().
// This merely tests the error conditions.
TEST_F(GeometryStateTest, AddRendererError) {
  const string kName = "unique";
  DRAKE_EXPECT_NO_THROW(
      geometry_state_.AddRenderer(kName, make_unique<DummyRenderEngine>()));

  // Non-unique name.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.AddRenderer(kName, make_unique<DummyRenderEngine>()),
      fmt::format("AddRenderer..: A renderer with the name '{}' already exists",
                  kName));

  EXPECT_EQ(geometry_state_.GetRenderEngineByName("invliad_name"), nullptr);
  const render::RenderEngine* engine =
      geometry_state_.GetRenderEngineByName(kName);
  EXPECT_NE(engine, nullptr);
  EXPECT_NE(dynamic_cast<const DummyRenderEngine*>(engine), nullptr);
}

// Tests that when assigning a geometry the perception role, that the process
// respects the ("renderer", "accepting") property with the following semantics:
//  1. Missing property --> all renderers accept the geometry.
//  2. Property has empty set --> all renderers accept the geometry.
//  3. Property has non-empty set --> only renderers with matching names accept
//     the geometries.
TEST_F(GeometryStateTest, RespectAcceptingRendererProperty) {
  SetUpSingleSourceTree();

  ASSERT_TRUE(geometry_state_.HasRenderer(kDummyRenderName));
  const DummyRenderEngine& first_renderer =
      dynamic_cast<const DummyRenderEngine&>(
          gs_tester_.GetRenderEngineOrThrow(kDummyRenderName));

  // We should already have a renderer with the name: kDummyRenderName. Add a
  // new renderer.
  auto new_renderer = make_unique<DummyRenderEngine>();
  const DummyRenderEngine& second_renderer = *new_renderer.get();
  const string second_name = "second_renderer";
  geometry_state_.AddRenderer(second_name, std::move(new_renderer));
  ASSERT_TRUE(geometry_state_.HasRenderer(second_name));

  ASSERT_EQ(first_renderer.num_registered(), 0);
  ASSERT_EQ(second_renderer.num_registered(), 0);

  // Instantiate base properties with the minimum acceptable set of properties.
  PerceptionProperties base_properties =
      DummyRenderEngine().accepting_properties();
  base_properties.AddProperty("phong", "diffuse",
                              Vector4<double>{0.8, 0.8, 0.8, 1.0});
  base_properties.AddProperty("label", "id", RenderLabel::kDontCare);
  {
    // Case: No property provided; geometry belongs to all.
    const GeometryId id = geometries_[0];
    geometry_state_.AssignRole(source_id_, id, base_properties);
    EXPECT_TRUE(first_renderer.is_registered(id));
    EXPECT_TRUE(second_renderer.is_registered(id));
  }

  {
    // Case: Property provided with empty set of names.
    PerceptionProperties properties(base_properties);
    properties.AddProperty("renderer", "accepting", set<string>{});
    const GeometryId id = geometries_[1];
    geometry_state_.AssignRole(source_id_, id, std::move(properties));
    EXPECT_TRUE(first_renderer.is_registered(id));
    EXPECT_TRUE(second_renderer.is_registered(id));
  }

  {
    // Case: Property provided with a single name.
    PerceptionProperties properties(base_properties);
    properties.AddProperty("renderer", "accepting",
                           set<string>{kDummyRenderName});
    const GeometryId id = geometries_[2];
    geometry_state_.AssignRole(source_id_, id, std::move(properties));
    EXPECT_TRUE(first_renderer.is_registered(id));
    EXPECT_FALSE(second_renderer.is_registered(id));
  }

  {
    // Case: Property provided with other single name.
    PerceptionProperties properties(base_properties);
    properties.AddProperty("renderer", "accepting", set<string>{second_name});
    const GeometryId id = geometries_[3];
    geometry_state_.AssignRole(source_id_, id, std::move(properties));
    EXPECT_FALSE(first_renderer.is_registered(id));
    EXPECT_TRUE(second_renderer.is_registered(id));
  }

  {
    // Case: Property provided with other arbitrary name.
    PerceptionProperties properties(base_properties);
    properties.AddProperty("renderer", "accepting", set<string>{"junk"});
    const GeometryId id = geometries_[4];
    geometry_state_.AssignRole(source_id_, id, std::move(properties));
    EXPECT_FALSE(first_renderer.is_registered(id));
    EXPECT_FALSE(second_renderer.is_registered(id));
  }

  {
    // Case: Property provided with all names.
    PerceptionProperties properties(base_properties);
    properties.AddProperty("renderer", "accepting",
                           set<string>{kDummyRenderName, second_name});
    const GeometryId id = geometries_[5];
    geometry_state_.AssignRole(source_id_, id, std::move(properties));
    EXPECT_TRUE(first_renderer.is_registered(id));
    EXPECT_TRUE(second_renderer.is_registered(id));
  }
}

// A special case of "respecting the ("renderer", "accepting")" test. When the
// render engine(s) is(are) added *after* geometry is rendered, the geometries
// should still only be assigned to render engines which are declared
// "acceptable".
TEST_F(GeometryStateTest, PostHocRenderEngineRespectAcceptingRenderer) {
  // Geometries, by default, have no roles assigned.
  SetUpSingleSourceTree();

  // By construction, geometry_state_ has a render engine registered: named
  // kDummyRenderName. We'll ignore it for this test. Instead, we'll create a
  // new render engine with a new name and add it after the fact. Some of the
  // geometries will accept it, some won't.
  const string second_name = "second_renderer";

  enum AcceptState { kAccept, kReject, kEmpty, kNoStatement };

  // Instantiate properties. If `second_accepts` is true, the accepting renderer
  // will be the new renderer named `second_name`. Otherwise, it will be
  // accepted by a fake name associated with no renderer.
  auto make_properties = [&second_name](AcceptState acceptance) {
    PerceptionProperties properties =
        DummyRenderEngine().accepting_properties();

    properties.AddProperty("phong", "diffuse",
                           Vector4<double>{0.8, 0.8, 0.8, 1.0});
    properties.AddProperty("label", "id", RenderLabel::kDontCare);
    switch (acceptance) {
      case kAccept:
        properties.AddProperty("renderer", "accepting",
                               set<string>{second_name});
        break;
      case kReject:
        properties.AddProperty("renderer", "accepting", set<string>{"invalid"});
        break;
      case kEmpty:
        properties.AddProperty("renderer", "accepting", set<string>{});
        break;
      case kNoStatement:
        // Do nothing.
        break;
    }
    return properties;
  };

  // We'll assign the perception role to four geometries:
  //   - The first explicitly declares the new renderer as accepting.
  //   - The second explicitly declares an alternate renderer, excluding the
  //     new renderer.
  //   - The third declares an empty set of accepting renderers, equivalent
  //     to saying nothing. It is accepted by every renderer.
  //   - The fourth says nothing. It is accepted by every render.
  // Therefore, the new renderer should register three of the four geometries.
  geometry_state_.AssignRole(source_id_, geometries_[0],
                             make_properties(kAccept));
  geometry_state_.AssignRole(source_id_, geometries_[1],
                             make_properties(kReject));
  geometry_state_.AssignRole(source_id_, geometries_[2],
                             make_properties(kEmpty));
  geometry_state_.AssignRole(source_id_, geometries_[3],
                             make_properties(kNoStatement));

  // We should already have a renderer with the name: kDummyRenderName. Add a
  // new renderer.
  auto second_renderer_owned = make_unique<DummyRenderEngine>();
  const DummyRenderEngine& second_renderer = *second_renderer_owned.get();

  geometry_state_.AddRenderer(second_name, std::move(second_renderer_owned));
  ASSERT_TRUE(geometry_state_.HasRenderer(second_name));

  EXPECT_EQ(second_renderer.num_registered(), 3);
}

TEST_F(GeometryStateTest, GetRenderEngine) {
  const render::RenderEngine& engine =
      gs_tester_.GetRenderEngineOrThrow(kDummyRenderName);
  EXPECT_EQ(&engine, render_engine_);
  DRAKE_EXPECT_THROWS_MESSAGE(gs_tester_.GetRenderEngineOrThrow("bad name"),
                              "No renderer exists with name.*");
}

// Confirms that the renderer(s) have poses updated properly when
// FinalizePoseUpdate() is called.
TEST_F(GeometryStateTest, RendererPoseUpdate) {
  // Add a *second* render engine to make sure that *all* get updated.
  auto render_engine = make_unique<DummyRenderEngine>();
  DummyRenderEngine* second_engine = render_engine.get();
  const std::string second_engine_name = "second_engine";
  geometry_state_.AddRenderer(second_engine_name, std::move(render_engine));

  SetUpSingleSourceTree(Assign::kPerception);

  // Reality check -- all geometries report as part of both engines.
  for (int i = 0; i < single_tree_dynamic_rigid_geometry_count(); ++i) {
    const InternalGeometry* geometry = gs_tester_.GetGeometry(geometries_[i]);
    EXPECT_TRUE(render_engine_->has_geometry(geometry->id()));
    EXPECT_TRUE(second_engine->has_geometry(geometry->id()));
  }

  EXPECT_EQ(second_engine->updated_ids().size(), 0u);
  EXPECT_EQ(render_engine_->updated_ids().size(), 0u);

  // Set poses of frames to the initial values.
  FramePoseVector<double> poses;
  for (int f = 0; f < static_cast<int>(frames_.size()); ++f) {
    poses.set_value(frames_[f], X_PFs_[f]);
  }
  gs_tester_.SetFramePoses(source_id_, poses,
                           &gs_tester_.mutable_kinematics_data());
  gs_tester_.FinalizePoseUpdate();

  // Confirm poses between two GeometryId -> Pose maps.
  auto expect_poses = [](const auto& test, const auto& expected) {
    ASSERT_EQ(test.size(), expected.size());
    for (const auto& expected_id_pose_pair : expected) {
      const GeometryId expected_id = expected_id_pose_pair.first;
      const auto& test_iter = test.find(expected_id);
      EXPECT_NE(test_iter, test.end());
      EXPECT_TRUE(
          CompareMatrices(test_iter->second.GetAsMatrix34(),
                          expected_id_pose_pair.second.GetAsMatrix34()));
    }
  };

  auto get_expected_ids = [this]() {
    map<GeometryId, RigidTransformd> expected;
    for (int i = 0; i < single_tree_dynamic_rigid_geometry_count(); ++i) {
      const GeometryId id = geometries_[i];
      expected.insert({id, gs_tester_.get_geometry_world_poses().at(id)});
    }
    return expected;
  };

  map<GeometryId, RigidTransformd> expected_ids = get_expected_ids();
  expect_poses(second_engine->updated_ids(), expected_ids);
  expect_poses(render_engine_->updated_ids(), expected_ids);
  render_engine_->init_test_data();
  second_engine->init_test_data();

  // Set poses of frames to an alternate value - fixed offset from initial
  // values.
  const Vector3d offset{1, 2, 3};
  for (int f = 0; f < static_cast<int>(frames_.size()); ++f) {
    RigidTransformd X_PF = X_PFs_[f];
    X_PF.set_translation(X_PF.translation() + offset);
    poses.set_value(frames_[f], X_PF);
  }
  EXPECT_EQ(second_engine->updated_ids().size(), 0u);
  EXPECT_EQ(render_engine_->updated_ids().size(), 0u);
  gs_tester_.SetFramePoses(source_id_, poses,
                           &gs_tester_.mutable_kinematics_data());
  gs_tester_.FinalizePoseUpdate();

  // Confirm poses.
  expected_ids = get_expected_ids();
  expect_poses(second_engine->updated_ids(), expected_ids);
  expect_poses(render_engine_->updated_ids(), expected_ids);
}

TEST_F(GeometryStateTest, DrivenMeshData) {
  SourceId s_id = NewSource();
  const Sphere sphere(1.0);
  // Make the resolution hint large enough so we end up with an octahedron.
  constexpr double kRezHint = 10.0;
  auto instance = make_unique<GeometryInstance>(
      RigidTransformd::Identity(), make_unique<Sphere>(sphere), "sphere");
  GeometryId g_id = geometry_state_.RegisterDeformableGeometry(
      s_id, InternalFrame::world_frame_id(), std::move(instance), kRezHint);
  geometry_state_.AssignRole(s_id, g_id, PerceptionProperties());
  geometry_state_.AssignRole(s_id, g_id, IllustrationProperties());
  geometry_state_.AssignRole(s_id, g_id, ProximityProperties());

  {
    // We expect one driven geometry that's the surface mesh of the control
    // mesh.
    for (const auto role : std::vector<Role>{
             Role::kProximity, Role::kIllustration, Role::kPerception}) {
      const internal::DrivenMeshData& data = gs_tester_.driven_mesh_data(role);
      EXPECT_TRUE(data.driven_meshes().contains(g_id));
      EXPECT_EQ(data.driven_meshes().at(g_id).size(), 1);
      const auto& driven_mesh = data.driven_meshes().at(g_id)[0];
      EXPECT_EQ(driven_mesh.num_control_vertices(), 7);
      EXPECT_EQ(driven_mesh.triangle_surface_mesh().num_triangles(), 8);
    }

    // Remove the perception role and the driven mesh is removed along with it.
    geometry_state_.RemoveRole(s_id, g_id, Role::kPerception);
    const internal::DrivenMeshData& data =
        gs_tester_.driven_mesh_data(Role::kPerception);
    EXPECT_FALSE(data.driven_meshes().contains(g_id));
  }

  {
    // Add the perception role back with a specified render geometry consisting
    // of two distinct meshes. Note that distinct RenderMesh instances are made
    // if there are more than one material when loading the obj file.
    const char mtl_file[] = "multi.mtl";
    WriteFile(R"""(
    newmtl test_material_1
    Kd 1.0 0.0 1.0

    newmtl test_material_2
    Kd 1.0 0.0 0.0
  )""",
              mtl_file);
    const std::string obj_path = WriteFile(fmt::format(R"""(
        mtllib {}
        v 0 0 0
        v 0.1 0.1 0.1
        v 0.2 0.2 0.2
        vn 0 1 0
        usemtl test_material_1
        f 1//1 2//1 3//1
        usemtl test_material_2
        f 2//1 3//1 1//1
        f 3//1 1//1 2//1)""",
                                                       mtl_file),
                                           "multi_mtl.obj");
    PerceptionProperties perception_properties;
    perception_properties.AddProperty("deformable", "embedded_mesh", obj_path);
    geometry_state_.AssignRole(s_id, g_id, perception_properties);
    const internal::DrivenMeshData& data =
        gs_tester_.driven_mesh_data(Role::kPerception);
    EXPECT_TRUE(data.driven_meshes().contains(g_id));
    EXPECT_EQ(data.driven_meshes().at(g_id).size(), 2);
  }
}

// Confirms that an accepting renderer
//  1. registers deformable geometries with perception roles,
//  2. updates configurations of registered deformable geometries when
// FinalizeConfigurationUpdate() is called,
//  3. removes the geometry when the perception role of the geometry is removed.
TEST_F(GeometryStateTest, RendererConfigurationUpdate) {
  SourceId s_id = NewSource();
  const Sphere sphere(1.0);
  // Make the resolution hint large enough so we end up with an octahedron.
  constexpr double kRezHint = 10.0;
  auto instance = make_unique<GeometryInstance>(
      RigidTransformd::Identity(), make_unique<Sphere>(sphere), "sphere");
  GeometryId g_id = geometry_state_.RegisterDeformableGeometry(
      s_id, InternalFrame::world_frame_id(), std::move(instance), kRezHint);

  PerceptionProperties properties = render_engine_->accepting_properties();
  properties.AddProperty("phong", "diffuse",
                         Vector4<double>{0.8, 0.8, 0.8, 1.0});
  properties.AddProperty("label", "id", RenderLabel::kDontCare);
  geometry_state_.AssignRole(s_id, g_id, properties);
  // Test object 1.
  EXPECT_TRUE(render_engine_->has_geometry(g_id));

  const std::vector<VectorXd> old_q_WG =
      render_engine_->world_configurations(g_id);
  const std::vector<VectorXd> old_nhat_W = render_engine_->world_normals(g_id);

  // We know that the deformable octahedron has 7 vertices.
  const int kNumControlMeshVertices = 7;
  gs_tester_.mutable_kinematics_data().q_WGs[g_id] =
      VectorXd::LinSpaced(3 * kNumControlMeshVertices, 0.0, 1.0);
  gs_tester_.FinalizeConfigurationUpdate();
  const std::vector<VectorXd> new_q_WG =
      render_engine_->world_configurations(g_id);
  const std::vector<VectorXd> new_nhat_W = render_engine_->world_normals(g_id);

  // Test objective 2.
  // Test that the vertex positions and normals have been modified. The
  // correctness of these updates has been tested in the tests for
  // DrivenTriangleMesh and RenderEngine.
  EXPECT_FALSE(CompareMatrices(old_q_WG[0], new_q_WG[0], 0.1));
  EXPECT_FALSE(CompareMatrices(old_nhat_W[0], new_nhat_W[0], 0.1));

  // Test objective 3.
  geometry_state_.RemoveRole(s_id, g_id, Role::kPerception);
  EXPECT_FALSE(render_engine_->has_geometry(g_id));
}

// This tests the equivalence of versions among copies of GeometryState
// instances; two copies are equivalent and equivalence is transitive. So, for
// state s: copy(s) == s and copy(copy(s)) == s.
TEST_F(GeometryStateTest, GeometryVersionCopies) {
  SetUpSingleSourceTree(Assign::kProximity);
  // Create a few copies of geometry state with the following tree of create
  // where a child node is copied from the parent node.
  //
  //                 geometry_state_
  //                ________|________
  //                |               |
  //               gs1              g2
  //                |
  //               gs3
  //
  // Verify that they are share the exact same version for all roles.
  GeometryState<double> gs1(geometry_state_);
  GeometryState<double> gs2(geometry_state_);
  GeometryState<double> gs3(gs1);
  VerifyIdenticalVersions(gs1, geometry_state_);
  VerifyIdenticalVersions(gs2, geometry_state_);
  VerifyIdenticalVersions(gs1, gs2);
  VerifyIdenticalVersions(gs3, geometry_state_);
  VerifyIdenticalVersions(gs3, gs1);
  VerifyIdenticalVersions(gs3, gs2);
}

// Confirms that geometry_version_ is updated correctly in each public method of
// geometry state. Every (non-const) API in GeometryState should be added to
// this test function so we can fully characterize every API's effect on
// geometry version.
TEST_F(GeometryStateTest, GeometryVersionUpdate) {
  SetUpSingleSourceTree();

  SourceId new_source = VerifyVersionUnchanged(
      &GeometryState<double>::RegisterNewSource, "my_new_source");
  // Registering a new frame does not modify the versions.
  FrameId new_frame_0 = VerifyVersionUnchanged(
      static_cast<FrameId (GeometryState<double>::*)(  // NOLINT
          SourceId, const GeometryFrame&)>(
          &GeometryState<double>::RegisterFrame),
      new_source, GeometryFrame("new_f0"));
  VerifyVersionUnchanged(
      static_cast<FrameId (GeometryState<double>::*)(  // NOLINT
          SourceId, FrameId, const GeometryFrame&)>(
          &GeometryState<double>::RegisterFrame),
      new_source, new_frame_0, GeometryFrame("new_f1"));

  // Registering geometries with no roles assigned does not change the versions.
  VerifyVersionUnchanged(
      &GeometryState<double>::RegisterGeometry, new_source, new_frame_0,
      std::make_unique<GeometryInstance>(
          RigidTransformd(), make_unique<Sphere>(1), "new_geometry_0"));
  VerifyVersionUnchanged(
      &GeometryState<double>::RegisterAnchoredGeometry, new_source,
      std::make_unique<GeometryInstance>(
          RigidTransformd(), make_unique<Sphere>(1), "new_geometry_2"));

  // Adding a new proximity role or replacing a proximity role modifies the
  // proximity version, but not the other versions.
  VerifyRoleVersionModified(
      Role::kProximity,
      static_cast<void (GeometryState<double>::*)(  // NOLINT
          SourceId, GeometryId, ProximityProperties, RoleAssign)>(
          &GeometryState<double>::AssignRole),
      source_id_, geometries_[0], ProximityProperties(), RoleAssign::kNew);

  VerifyRoleVersionModified(
      Role::kProximity,
      static_cast<void (GeometryState<double>::*)(  // NOLINT
          SourceId, GeometryId, ProximityProperties, RoleAssign)>(
          &GeometryState<double>::AssignRole),
      source_id_, geometries_[0], ProximityProperties(), RoleAssign::kReplace);

  // Add a new perception role that is accepted by a renderer modifies the
  // perception version, but not the other versions.
  PerceptionProperties base_perception_properties =
      DummyRenderEngine().accepting_properties();
  base_perception_properties.AddProperty("phong", "diffuse",
                                         Vector4<double>{0.8, 0.8, 0.8, 1.0});
  base_perception_properties.AddProperty("label", "id", RenderLabel::kDontCare);
  {
    PerceptionProperties perception_properties(base_perception_properties);
    perception_properties.AddProperty("renderer", "accepting",
                                      set<string>{kDummyRenderName});
    VerifyRoleVersionModified(
        Role::kPerception,
        static_cast<void (GeometryState<double>::*)(  // NOLINT
            SourceId, GeometryId, PerceptionProperties, RoleAssign)>(
            &GeometryState<double>::AssignRole),
        source_id_, geometries_[1], perception_properties, RoleAssign::kNew);
  }

  // If the perception property is not accepted by any renderer, the perception
  // version is not modified.
  {
    PerceptionProperties perception_properties(base_perception_properties);
    perception_properties.AddProperty("renderer", "accepting",
                                      set<string>{"junk"});
    VerifyVersionUnchanged(
        static_cast<void (GeometryState<double>::*)(  // NOLINT
            SourceId, GeometryId, PerceptionProperties, RoleAssign)>(
            &GeometryState<double>::AssignRole),
        source_id_, geometries_[2], perception_properties, RoleAssign::kNew);
  }

  // Adding a new illustration role modifies the illustration version, but not
  // the other versions.
  IllustrationProperties illustration_properties;
  illustration_properties.AddProperty("phong", "diffuse",
                                      Vector4<double>{0.8, 0.8, 0.8, 1.0});
  VerifyRoleVersionModified(
      Role::kIllustration,
      static_cast<void (GeometryState<double>::*)(  // NOLINT
          SourceId, GeometryId, IllustrationProperties, RoleAssign)>(
          &GeometryState<double>::AssignRole),
      source_id_, geometries_[3], illustration_properties, RoleAssign::kNew);

  // Removing a proximity role modifies the proximity version but not the other
  // versions.
  VerifyRoleVersionModified(
      Role::kProximity,
      static_cast<int (GeometryState<double>::*)(SourceId, GeometryId, Role)>(
          &GeometryState<double>::RemoveRole),
      source_id_, geometries_[0], Role::kProximity);

  // Removing a perception role from a geometry registered in a renderer
  // modifies the perception version but not the other versions.
  VerifyRoleVersionModified(
      Role::kPerception,
      static_cast<int (GeometryState<double>::*)(SourceId, GeometryId, Role)>(
          &GeometryState<double>::RemoveRole),
      source_id_, geometries_[1], Role::kPerception);

  // Removing the perception role from a geometry not registered in any renderer
  // does not modify any version.
  VerifyVersionUnchanged(
      static_cast<int (GeometryState<double>::*)(SourceId, GeometryId, Role)>(
          &GeometryState<double>::RemoveRole),
      source_id_, geometries_[2], Role::kPerception);

  // Removing a illustration role modifies the illustration version but not the
  // other versions.
  VerifyRoleVersionModified(
      Role::kIllustration,
      static_cast<int (GeometryState<double>::*)(SourceId, GeometryId, Role)>(
          &GeometryState<double>::RemoveRole),
      source_id_, geometries_[3], Role::kIllustration);

  // Removing a non-existing role does not modify the versions.
  VerifyVersionUnchanged(
      static_cast<int (GeometryState<double>::*)(SourceId, GeometryId, Role)>(
          &GeometryState<double>::RemoveRole),
      source_id_, geometries_[3], Role::kIllustration);
  VerifyVersionUnchanged(
      static_cast<int (GeometryState<double>::*)(SourceId, GeometryId, Role)>(
          &GeometryState<double>::RemoveRole),
      source_id_, geometries_[3], Role::kPerception);

  // Removing a geometry without any perception role from a renderer does not
  // modify any version.
  VerifyVersionUnchanged(static_cast<int (GeometryState<double>::*)(  // NOLINT
                             const std::string&, SourceId, GeometryId)>(
                             &GeometryState<double>::RemoveFromRenderer),
                         kDummyRenderName, source_id_, geometries_[1]);

  // Add the perception property back on geometries_[1].
  PerceptionProperties perception_properties(base_perception_properties);
  perception_properties.AddProperty("renderer", "accepting",
                                    set<string>{kDummyRenderName, "second"});
  geometry_state_.AssignRole(source_id_, geometries_[1], perception_properties,
                             RoleAssign::kNew);

  // Removing a geometry with perception role from a renderer does modify
  // the perception version.
  VerifyRoleVersionModified(
      Role::kPerception,
      static_cast<int (GeometryState<double>::*)(  // NOLINT
          const std::string&, SourceId, GeometryId)>(
          &GeometryState<double>::RemoveFromRenderer),
      kDummyRenderName, source_id_, geometries_[1]);

  // Assign proximity role to the first three geometries to test versions on
  // proximity filters.
  for (int i = 0; i < 3; ++i) {
    geometry_state_.AssignRole(source_id_, geometries_[i],
                               ProximityProperties(), RoleAssign::kNew);
  }

  VerifyRoleVersionModified(Role::kProximity,
                            &GeometryState<double>::collision_filter_manager);

  // Note that geometries_[1] has perception role now.
  // When there exist geometries with perception properties, adding a renderer
  // that accepts those geometries modifies the perception version.
  VerifyRoleVersionModified(Role::kPerception,
                            &GeometryState<double>::AddRenderer, "second",
                            make_unique<DummyRenderEngine>());

  // Remove the perception role of the only two geometries that have perception
  // roles.
  geometry_state_.RemoveRole(source_id_, geometries_[1], Role::kPerception);
  // Adding a renderer when there's no geometry with perception role does not
  // modify version.
  VerifyVersionUnchanged(&GeometryState<double>::AddRenderer, "third",
                         make_unique<DummyRenderEngine>());
}

// The ComputeSignedDistancePairClosestPoints() API requires the user to provide
// GeometryIds, increasing the odds that they pick and pass the *wrong* ids.
// Rather than simply declaring "wrong", we attempt to give them additional
// guidance about what way it is wrong -- users have been confused about why
// an apparently valid GeometryId is rejected. The answer: it has the wrong
// role. Let's confirm we're getting the messages we expect.
TEST_F(GeometryStateTest, ComputeSignedDistancePairClosestPointsError) {
  SetUpSingleSourceTree();
  // GeometryIds 0 & 1 get proximity properties.
  const GeometryId proximity1 = geometries_[0];
  geometry_state_.AssignRole(source_id_, proximity1, ProximityProperties());
  const GeometryId proximity2 = geometries_[1];
  geometry_state_.AssignRole(source_id_, proximity2, ProximityProperties());
  // GeometryId 2 gets illustration.
  const GeometryId illus = geometries_[2];
  geometry_state_.AssignRole(source_id_, illus, IllustrationProperties());
  // GeometryId 3 gets perception.
  const GeometryId percep = geometries_[3];
  PerceptionProperties p = render_engine_->accepting_properties();
  p.AddProperty("label", "id", RenderLabel(10));
  geometry_state_.AssignRole(source_id_, percep, p);
  // Geometry 4 has no role.
  const GeometryId no_role = geometries_[4];
  const GeometryId bad_id = GeometryId::get_new_id();

  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.ComputeSignedDistancePairClosestPoints(proximity1,
                                                             bad_id),
      ".*has not been registered.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.ComputeSignedDistancePairClosestPoints(bad_id,
                                                             proximity1),
      ".*has not been registered.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.ComputeSignedDistancePairClosestPoints(no_role,
                                                             proximity1),
      ".*has no role.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.ComputeSignedDistancePairClosestPoints(proximity1,
                                                             no_role),
      ".*has no role.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.ComputeSignedDistancePairClosestPoints(proximity1, illus),
      ".*has the illustration role.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.ComputeSignedDistancePairClosestPoints(illus, proximity1),
      ".*has the illustration role.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.ComputeSignedDistancePairClosestPoints(proximity1,
                                                             percep),
      ".*has the perception role.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.ComputeSignedDistancePairClosestPoints(percep,
                                                             proximity1),
      ".*has the perception role.");
}

// GeometryState must convert GeometrySet to a set of GeometryIds for proximity
// engine. This will confirm that the transformation takes place.
TEST_F(GeometryStateTest, ComputeSignedDistanceGeometryToPoint) {
  SetUpWithRigidAndDeformableGeometries(Assign::kProximity);

  // This set includes rigid geometries 0, 1, 2, and anchored. Rigid 0 is
  // specified redundantly (explicitly and as part of frame 0). Rigid geometry 1
  // also comes from frame 0.
  const GeometrySet ids({geometries_[0], geometries_[2], anchored_geometry_},
                        {frames_[0]});
  // Confirm expected ordering.
  DRAKE_DEMAND(geometries_.back() < anchored_geometry_);
  DRAKE_DEMAND(anchored_geometry_ < deformable_geometries_[0]);

  const std::vector<SignedDistanceToPoint<double>> result =
      geometry_state_.ComputeSignedDistanceGeometryToPoint(Vector3d::Zero(),
                                                           ids);
  ASSERT_EQ(result.size(), 4);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(result[i].id_G, geometries_[i]);
  }
  EXPECT_EQ(result[3].id_G, anchored_geometry_);

  // Deformable geometries throw. Note: we're testing this explicitly because
  // a small change to GeometryState could cause the deformable geometry to
  // simply be ignored; we want to make sure that doesn't happen.
  const GeometrySet bad_id(deformable_geometries_[0]);
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.ComputeSignedDistanceGeometryToPoint(Vector3d::Zero(),
                                                           bad_id),
      ".*does not reference a geometry.*signed distance query");
}

// This is the base for testing the ApplyProximityDefaults() overloaded
// methods. Note that only the resulting proximity properties contents are
// tested.  Since we know (glass-box knowledge) that ApplyProximityDefaults
// calls GeometryState::AssignRole, we don't bother testing whether the
// proximity engine gets appropriately updated.
class ApplyProximityDefaultsTests : public testing::Test {
 protected:
  std::unique_ptr<ProximityProperties> MakeFullProperties() {
    // Fill the properties with identifiably stupid values.
    auto props = std::make_unique<ProximityProperties>();
    props->AddProperty(kHydroGroup, kComplianceType, HydroelasticType::kRigid);
    props->AddProperty(kHydroGroup, kElastic, 123.0);
    props->AddProperty(kHydroGroup, kRezHint, 456.0);
    props->AddProperty(kHydroGroup, kSlabThickness, 789.0);
    multibody::CoulombFriction<double> friction{2.2, 2.2};
    props->AddProperty(kMaterialGroup, kFriction, friction);
    props->AddProperty(kMaterialGroup, kHcDissipation, 987.0);
    props->AddProperty(kMaterialGroup, kRelaxationTime, 654.0);
    props->AddProperty(kMaterialGroup, kPointStiffness, 321.0);
    return props;
  }

  GeometryId AddSphere(const std::string name,
                       const ProximityProperties* props) {
    auto instance = std::make_unique<GeometryInstance>(math::RigidTransformd{},
                                                       Sphere(0.1), name);
    if (props != nullptr) {
      instance->set_proximity_properties(*props);
    }
    return geometry_state_.RegisterGeometry(source_id_, frame_id_,
                                            std::move(instance));
  }

  DefaultProximityProperties empty_defaults_{"undefined", {}, {}, {}, {},
                                             {},          {}, {}, {}, {}};
  // Use stupid values distinct from those in MakeFullProperties().
  DefaultProximityProperties full_defaults_{"compliant", 0.1, 0.2, 0.3, 0.4,
                                            0.4,         0.5, 0.6, 0.7, 0.8};
  GeometryState<double> geometry_state_;
  SourceId source_id_{geometry_state_.RegisterNewSource("test")};
  FrameId frame_id_{
      geometry_state_.RegisterFrame(source_id_, GeometryFrame("frame"))};
};

TEST_F(ApplyProximityDefaultsTests, TrivialApplyProximityDefaults) {
  // Feeding in an empty geometry state does not crash.
  EXPECT_NO_THROW(geometry_state_.ApplyProximityDefaults({}));
}

TEST_F(ApplyProximityDefaultsTests, NoRole) {
  // Geometries without a proximity role are unaffected.
  auto geometry_id = AddSphere("no_prox_role", nullptr);
  geometry_state_.ApplyProximityDefaults(full_defaults_);
  EXPECT_EQ(geometry_state_.GetProximityProperties(geometry_id), nullptr);
}

TEST_F(ApplyProximityDefaultsTests, EmptyPropsEmptyDefaults) {
  // Given empty properties and minimal defaults, only compliance type is
  // populated.
  ProximityProperties empty_props;
  auto geometry_id = AddSphere("empty_props", &empty_props);
  geometry_state_.ApplyProximityDefaults(empty_defaults_, geometry_id);
  auto* props = geometry_state_.GetProximityProperties(geometry_id);
  ASSERT_NE(props, nullptr);
  EXPECT_EQ(props->GetProperty<HydroelasticType>(kHydroGroup, kComplianceType),
            HydroelasticType::kUndefined);
  EXPECT_FALSE(props->HasProperty(kHydroGroup, kElastic));
  EXPECT_FALSE(props->HasProperty(kHydroGroup, kRezHint));
  EXPECT_FALSE(props->HasProperty(kHydroGroup, kSlabThickness));
  EXPECT_FALSE(props->HasProperty(kMaterialGroup, kFriction));
  EXPECT_FALSE(props->HasProperty(kMaterialGroup, kHcDissipation));
  EXPECT_FALSE(props->HasProperty(kMaterialGroup, kRelaxationTime));
  EXPECT_FALSE(props->HasProperty(kMaterialGroup, kPointStiffness));
}

TEST_F(ApplyProximityDefaultsTests, EmptyPropsFullDefaults) {
  // Given empty properties and full defaults, the default struct values are
  // written into the properties.
  ProximityProperties empty_props;
  auto geometry_id = AddSphere("empty_props", &empty_props);
  geometry_state_.ApplyProximityDefaults(full_defaults_, geometry_id);
  auto* props = geometry_state_.GetProximityProperties(geometry_id);
  ASSERT_NE(props, nullptr);
  EXPECT_EQ(
      props->GetProperty<HydroelasticType>(kHydroGroup, kComplianceType),
      internal::GetHydroelasticTypeFromString(full_defaults_.compliance_type));
  EXPECT_EQ(props->GetProperty<double>(kHydroGroup, kElastic),
            *full_defaults_.hydroelastic_modulus);
  EXPECT_EQ(props->GetProperty<double>(kHydroGroup, kRezHint),
            *full_defaults_.resolution_hint);
  EXPECT_EQ(props->GetProperty<double>(kHydroGroup, kSlabThickness),
            *full_defaults_.slab_thickness);

  auto friction = props->GetProperty<multibody::CoulombFriction<double>>(
      kMaterialGroup, kFriction);
  EXPECT_EQ(friction.static_friction(), *full_defaults_.static_friction);
  EXPECT_EQ(friction.dynamic_friction(), *full_defaults_.dynamic_friction);
  EXPECT_EQ(props->GetProperty<double>(kMaterialGroup, kHcDissipation),
            *full_defaults_.hunt_crossley_dissipation);
  EXPECT_EQ(props->GetProperty<double>(kMaterialGroup, kRelaxationTime),
            *full_defaults_.relaxation_time);
  EXPECT_EQ(props->GetProperty<double>(kMaterialGroup, kPointStiffness),
            *full_defaults_.point_stiffness);
}

TEST_F(ApplyProximityDefaultsTests, FullPropsFullDefaults) {
  // Given full properties and full defaults, the property values remain
  // unchanged.
  std::unique_ptr<ProximityProperties> full_props = MakeFullProperties();
  auto geometry_id = AddSphere("full_props", full_props.get());
  geometry_state_.ApplyProximityDefaults(full_defaults_, geometry_id);
  auto* props = geometry_state_.GetProximityProperties(geometry_id);
  ASSERT_NE(props, nullptr);
  EXPECT_EQ(
      props->GetProperty<HydroelasticType>(kHydroGroup, kComplianceType),
      full_props->GetProperty<HydroelasticType>(kHydroGroup, kComplianceType));
  EXPECT_EQ(props->GetProperty<double>(kHydroGroup, kElastic),
            full_props->GetProperty<double>(kHydroGroup, kElastic));
  EXPECT_EQ(props->GetProperty<double>(kHydroGroup, kRezHint),
            full_props->GetProperty<double>(kHydroGroup, kRezHint));
  EXPECT_EQ(props->GetProperty<double>(kHydroGroup, kSlabThickness),
            full_props->GetProperty<double>(kHydroGroup, kSlabThickness));

  auto friction = props->GetProperty<multibody::CoulombFriction<double>>(
      kMaterialGroup, kFriction);
  auto full_props_friction =
      full_props->GetProperty<multibody::CoulombFriction<double>>(
          kMaterialGroup, kFriction);
  EXPECT_EQ(friction.static_friction(), full_props_friction.static_friction());
  EXPECT_EQ(friction.dynamic_friction(),
            full_props_friction.dynamic_friction());
  EXPECT_EQ(props->GetProperty<double>(kMaterialGroup, kHcDissipation),
            full_props->GetProperty<double>(kMaterialGroup, kHcDissipation));
  EXPECT_EQ(props->GetProperty<double>(kMaterialGroup, kRelaxationTime),
            full_props->GetProperty<double>(kMaterialGroup, kRelaxationTime));
  EXPECT_EQ(props->GetProperty<double>(kMaterialGroup, kPointStiffness),
            full_props->GetProperty<double>(kMaterialGroup, kPointStiffness));
}

TEST_F(ApplyProximityDefaultsTests, MultipleGeometries) {
  // The one-argument overload modifies multiple geometries, consistent with
  // the detailed behavior tested above.
  ProximityProperties empty_props;
  const int kNumTestGeoms = 10;
  std::vector<GeometryId> geometry_ids;
  geometry_ids.reserve(kNumTestGeoms);
  for (int k = 0; k < kNumTestGeoms; ++k) {
    geometry_ids.push_back(AddSphere(fmt::to_string(k), &empty_props));
  }
  geometry_state_.ApplyProximityDefaults(full_defaults_);
  // Spot-check that values got written into the properties.
  for (const auto& id : geometry_ids) {
    auto* props = geometry_state_.GetProximityProperties(id);
    ASSERT_NE(props, nullptr);
    EXPECT_EQ(props->GetProperty<double>(kMaterialGroup, kPointStiffness),
              *full_defaults_.point_stiffness);
  }
}

// Test the ability of GeometryState to successfully report geometries with
// *mesh* hydroelastic representations. We test the following cases:
//   Case: invalid id.
//   Case: id doesn't have proximity role.
//   Case: id has proximity role, but no hydro properties.
//   Case: id has surface mesh.
//   Case: id has volume mesh.
//   Case: id is half space (has hydro representation, but not a mesh)
GTEST_TEST(GeometryStateHydroTest, GetHydroMesh) {
  GeometryState<double> geometry_state;
  const SourceId source_id = geometry_state.RegisterNewSource("hydro_test");

  ProximityProperties rigid_hydro;
  AddRigidHydroelasticProperties(1.0, &rigid_hydro);
  ProximityProperties soft_hydro;
  AddContactMaterial(0.0, {}, {}, &soft_hydro);
  AddCompliantHydroelasticProperties(1.0, 1e8, &soft_hydro);

  // We'll simply affix a number of geometries as anchored with the identity
  // pose. The other details don't really matter.
  const RigidTransformd X_WG;

  // Case: invalid id.
  {
    const GeometryId id = GeometryId::get_new_id();

    const auto maybe_mesh = geometry_state.maybe_get_hydroelastic_mesh(id);
    EXPECT_EQ(maybe_mesh.index(), 0);
  }

  // Case: id doesn't have proximity role.
  {
    const GeometryId id = geometry_state.RegisterAnchoredGeometry(
        source_id, make_unique<GeometryInstance>(X_WG, make_unique<Sphere>(1),
                                                 "no_proximity"));

    const auto maybe_mesh = geometry_state.maybe_get_hydroelastic_mesh(id);
    EXPECT_EQ(maybe_mesh.index(), 0);
  }

  // Case: id has proximity role, but no hydro properties.
  {
    const GeometryId id = geometry_state.RegisterAnchoredGeometry(
        source_id, make_unique<GeometryInstance>(X_WG, make_unique<Sphere>(1),
                                                 "no_hydro"));
    geometry_state.AssignRole(source_id, id, ProximityProperties());

    const auto maybe_mesh = geometry_state.maybe_get_hydroelastic_mesh(id);
    EXPECT_EQ(maybe_mesh.index(), 0);
  }

  // Case: id has surface mesh.
  {
    const GeometryId id = geometry_state.RegisterAnchoredGeometry(
        source_id, make_unique<GeometryInstance>(X_WG, make_unique<Sphere>(1),
                                                 "rigid_mesh"));
    geometry_state.AssignRole(source_id, id, rigid_hydro);

    const auto maybe_mesh = geometry_state.maybe_get_hydroelastic_mesh(id);
    EXPECT_TRUE(
        std::holds_alternative<const TriangleSurfaceMesh<double>*>(maybe_mesh));
    EXPECT_NE(std::get<const TriangleSurfaceMesh<double>*>(maybe_mesh),
              nullptr);
  }

  // Case: id has volume mesh.
  {
    const GeometryId id = geometry_state.RegisterAnchoredGeometry(
        source_id, make_unique<GeometryInstance>(X_WG, make_unique<Sphere>(1),
                                                 "soft_mesh"));
    geometry_state.AssignRole(source_id, id, soft_hydro);

    const auto maybe_mesh = geometry_state.maybe_get_hydroelastic_mesh(id);
    EXPECT_TRUE(std::holds_alternative<const VolumeMesh<double>*>(maybe_mesh));
    EXPECT_NE(std::get<const VolumeMesh<double>*>(maybe_mesh), nullptr);
  }

  // Case: id is half space (has hydro representation, but not a mesh)
  {
    const GeometryId id = geometry_state.RegisterAnchoredGeometry(
        source_id, make_unique<GeometryInstance>(X_WG, make_unique<HalfSpace>(),
                                                 "no_hydro_mesh"));
    geometry_state.AssignRole(source_id, id, rigid_hydro);

    const auto maybe_mesh = geometry_state.maybe_get_hydroelastic_mesh(id);
    EXPECT_TRUE(std::holds_alternative<std::monostate>(maybe_mesh));
  }
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
    // (geometry, renderer) pairs - i.e., removing a geometry from two renderers
    // still only reports one geometry's role has been removed.
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
                    !ids_without_role.contains(id));
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

INSTANTIATE_TEST_SUITE_P(GeometryStateTest, RemoveRoleTests,
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
      "Referenced geometry source \\d+ is not registered.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveRole(invalid_source_id, invalid_geometry_id,
                                 Role::kUnassigned),
      "Referenced geometry source \\d+ is not registered.");

  // Case: valid source id, but invalid frame/geometry id.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveRole(source_id_, invalid_frame_id,
                                 Role::kUnassigned),
      "Referenced .* frame doesn't belong to the source.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveRole(source_id_, invalid_geometry_id,
                                 Role::kUnassigned),

      "Referenced geometry \\d+ has not been registered.");

  // Case: frame/geometry id belongs to a different source.
  const SourceId source_id_2 =
      geometry_state_.RegisterNewSource("second_source");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveRole(source_id_2, frames_[0], Role::kUnassigned),
      "Referenced .* frame doesn't belong to the source.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveRole(source_id_2, geometries_[0],
                                 Role::kUnassigned),
      ".*the geometry doesn't belong to that source.");
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
  ASSERT_EQ(gs_tester_.RendererCount(), 0);
  EXPECT_TRUE(geometry.has_perception_role());

  EXPECT_EQ(
      geometry_state_.RemoveRole(source_id_, geometries_[0], Role::kPerception),
      1);
  EXPECT_FALSE(geometry.has_perception_role());
  EXPECT_EQ(
      geometry_state_.RemoveRole(source_id_, geometries_[0], Role::kPerception),
      0);
}

TEST_F(GeometryStateTest, ComputeAabbInWorld) {
  // Test invalid geometry id.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.ComputeAabbInWorld(GeometryId::get_new_id()),
      "No AABB available for invalid geometry id.*");

  const auto expect_aabb = [&](GeometryId id,
                               const Vector3d& expected_half_width,
                               const Vector3d& expected_center,
                               double tol = 1e-13) {
    const std::optional<Aabb> aabb = geometry_state_.ComputeAabbInWorld(id);
    ASSERT_TRUE(aabb.has_value());
    EXPECT_TRUE(CompareMatrices(aabb->half_width(), expected_half_width, tol));
    EXPECT_TRUE(CompareMatrices(aabb->center(), expected_center, tol));
  };

  const Sphere sphere(1.0);
  const auto register_deformable = [&](const std::string& name,
                                       bool add_proximity_role) {
    auto instance = make_unique<GeometryInstance>(
        RigidTransformd::Identity(), make_unique<Sphere>(sphere), name);
    if (add_proximity_role) {
      instance->set_proximity_properties(ProximityProperties());
    }
    return geometry_state_.RegisterDeformableGeometry(
        source_id_, InternalFrame::world_frame_id(), std::move(instance), 1.0);
  };

  // Test with non-deformable geometry.
  SetUpSingleSourceTree(Assign::kProximity);
  const GeometryId rigid_geom_id = geometries_[0];
  EXPECT_FALSE(geometry_state_.IsDeformableGeometry(rigid_geom_id));
  EXPECT_EQ(geometry_state_.ComputeAabbInWorld(rigid_geom_id), std::nullopt);

  // Test with deformable geometry with proximity role.
  const GeometryId deformable_id = register_deformable("deformable", true);
  expect_aabb(deformable_id, Vector3d::Ones(), Vector3d::Zero());

  // Test with deformable geometry without proximity role.
  const GeometryId deformable_no_prox_id =
      register_deformable("deformable_no_prox", false);
  expect_aabb(deformable_no_prox_id, Vector3d::Ones(), Vector3d::Zero());

  // Test deforming the geometry.
  const VectorX<double> original_q =
      geometry_state_.get_configurations_in_world(deformable_id);
  VectorX<double> new_q = original_q;
  // Move all vertices to (0.5, 0.5, 0.5)...
  for (int i = 0; i < new_q.size() / 3; ++i) {
    new_q.segment<3>(3 * i) = Vector3d(0.5, 0.5, 0.5);
  }
  // ...and move two vertices to (1, 1, 1) and (0, 0, 0) so that the AABB
  // is the unit cube centered at (0.5, 0.5, 0.5).
  new_q.segment<3>(0) = Vector3d(1, 1, 1);
  new_q.segment<3>(3) = Vector3d(0, 0, 0);

  GeometryConfigurationVector<double> configurations;
  configurations.set_value(deformable_id, new_q);
  configurations.set_value(deformable_no_prox_id, new_q);
  gs_tester_.SetGeometryConfiguration(source_id_, configurations,
                                      &gs_tester_.mutable_kinematics_data());
  gs_tester_.FinalizeConfigurationUpdate();

  expect_aabb(deformable_id, Vector3d(0.5, 0.5, 0.5), Vector3d(0.5, 0.5, 0.5));
  expect_aabb(deformable_no_prox_id, Vector3d(0.5, 0.5, 0.5),
              Vector3d(0.5, 0.5, 0.5));
}

TEST_F(GeometryStateTest, ComputeObbInWorld) {
  // Test invalid geometry id.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.ComputeObbInWorld(GeometryId::get_new_id()),
      "No OBB available for invalid geometry id.*");

  SetUpSingleSourceTree();

  // Helper to push poses to GeometryState and finalize.
  auto apply_poses = [&](const FramePoseVector<double>& poses) {
    gs_tester_.SetFramePoses(source_id_, poses,
                             &gs_tester_.mutable_kinematics_data());
    gs_tester_.FinalizePoseUpdate();
  };

  // Helper to check the OBB against expected results.
  constexpr double kTol = 1e-13;
  auto expect_obb = [&](GeometryId id, const RigidTransformd& X_WG) {
    const std::optional<Obb> obb = geometry_state_.ComputeObbInWorld(id);
    EXPECT_TRUE(obb.has_value());
    EXPECT_TRUE(CompareMatrices(obb->half_width(), Vector3d::Ones(), kTol));
    EXPECT_TRUE(
        CompareMatrices(obb->pose().translation(), X_WG.translation(), kTol));
    const RigidTransformd& X_GB =
        geometry_state_.GetObbInGeometryFrame(id)->pose();
    const RigidTransformd X_WB = X_WG * X_GB;
    EXPECT_TRUE(CompareMatrices(obb->pose().rotation().matrix(),
                                X_WB.rotation().matrix(), kTol));
  };

  const Sphere sphere(1.0);
  auto deformable_instance = make_unique<GeometryInstance>(
      RigidTransformd::Identity(), make_unique<Sphere>(sphere), "deformable");
  const GeometryId deformable_id = geometry_state_.RegisterDeformableGeometry(
      source_id_, InternalFrame::world_frame_id(),
      std::move(deformable_instance), 1.0);
  // The computation of OBB for deformable meshes involves a PCA and is hard to
  // write down, and the code paths is covered by other tests, so here we just
  // verify some OBB is produced.
  std::optional<Obb> obb = geometry_state_.ComputeObbInWorld(deformable_id);
  ASSERT_TRUE(obb.has_value());

  // Rigid geometry (Sphere) that supports OBBs.
  const GeometryId sphere_id = geometries_[0];
  ASSERT_FALSE(geometry_state_.IsDeformableGeometry(sphere_id));

  // Default frame poses.
  FramePoseVector<double> poses;
  for (int i = 0; i < static_cast<int>(frames_.size()); ++i) {
    poses.set_value(frames_[i], X_PFs_[i]);
  }
  apply_poses(poses);
  expect_obb(sphere_id, X_WFs_[0] * X_FGs_[0]);

  // Update frame pose and repeat.
  const RigidTransformd X_WF_new(math::RollPitchYawd(0.1, 0.2, 0.3),
                                 Vector3d(10.0, 20.0, 30.0));
  poses.set_value(frames_[0], X_WF_new);
  apply_poses(poses);
  expect_obb(sphere_id, X_WF_new * X_FGs_[0]);

  // Geometry that does not support OBBs (HalfSpace).
  const RigidTransformd X_WG = RigidTransformd::Identity();
  const GeometryId half_space_id = geometry_state_.RegisterGeometry(
      source_id_, frames_[0],
      make_unique<GeometryInstance>(X_WG, make_unique<HalfSpace>(),
                                    "half_space"));
  EXPECT_EQ(geometry_state_.ComputeObbInWorld(half_space_id), std::nullopt);
}

// GeometryState has three responsibilities when it comes to rendering:
//
//   1. Compute the correct camera pose in the world frame.
//   2. Handle finding render engines by name (including *not* finding them).
//   3. Call the appropriate RenderEngine API for the image type.
//
// For each of the Render*Image APIs, this establishes those three
// responsibilities are met.
class GeometryStateRenderTest : public ::testing::Test {
 protected:
  void SetUp() final {
    const SourceId s_id = geometry_state_.RegisterNewSource("render_test");

    // Add an arbitrarily posed frame (90° rotation around x-axis with non-zero
    // offset) to serve as the parent frame for the camera.
    X_WP_ = RigidTransformd(AngleAxis<double>(M_PI_2, Vector3d::UnitX()),
                            Vector3d{1, 2, 3});
    parent_id_ =
        geometry_state_.RegisterFrame(s_id, GeometryFrame("parent_frame"));

    // Equally arbitrary non-identity poses for the camera body in the parent
    // frame and camera sensor in the camera body frame.
    X_PC_ = RigidTransformd(AngleAxis<double>(M_PI_2, Vector3d::UnitY()),
                            Vector3d{2, 3, 4});
    X_CS_ = RigidTransformd(AngleAxis<double>(M_PI_2, Vector3d::UnitZ()),
                            Vector3d{3, 4, 5});
    // The expected pose of the camera *sensor* as passed to the render engine.
    X_WS_ = X_WP_ * X_PC_ * X_CS_;

    // Now add two render engines.
    auto engine1 = std::make_unique<DummyRenderEngine>();
    engine1_ = engine1.get();
    geometry_state_.AddRenderer("engine1", std::move(engine1));
    geometry_state_.AddRenderer("engine2",
                                std::make_unique<DummyRenderEngine>());
    // Make sure the geometry data knows the parent frame's pose.
    GeometryStateTester<double> tester;
    tester.set_state(&geometry_state_);
    FramePoseVector<double> poses;
    poses.set_value(parent_id_, X_WP_);
    tester.SetFramePoses(s_id, poses, &tester.mutable_kinematics_data());
  }

  static int width() { return 4; }
  static int height() { return 3; }
  render::RenderCameraCore camera_core(std::string_view engine_name) const {
    return render::RenderCameraCore(std::string(engine_name),
                                    {width(), height(), M_PI / 4}, {0.25, 10},
                                    X_CS_);
  }

  render::ColorRenderCamera color_camera(std::string_view engine_name) const {
    return render::ColorRenderCamera(camera_core(engine_name));
  }

  render::DepthRenderCamera depth_camera(std::string_view engine_name) const {
    return render::DepthRenderCamera(camera_core(engine_name), {1.0, 5.0});
  }

  GeometryState<double> geometry_state_;
  FrameId parent_id_{};
  // The pose of the *parent* frame P in world.
  RigidTransformd X_WP_;
  // The pose of the *camera* body frame C in the parent frame.
  RigidTransformd X_PC_;
  // The pose of the *sensor* frame S in the camera body frame.
  RigidTransformd X_CS_;
  // The concatenated pose of the sensor frame in the world.  See Setup().
  RigidTransformd X_WS_;
  DummyRenderEngine* engine1_{};
};

TEST_F(GeometryStateRenderTest, RenderColorImage) {
  // The image doesn't matter; it will *not* get written to. It just needs to be
  // compatible with the camera.
  systems::sensors::ImageRgba8U image(width(), height());

  // Case: valid render sets the viewport and calls the right API. We don't test
  //  to confirm that the image is passed along, that would be screamingly
  //  apparent.
  geometry_state_.RenderColorImage(color_camera("engine1"), parent_id_, X_PC_,
                                   &image);
  EXPECT_TRUE(CompareMatrices(engine1_->last_updated_X_WC().GetAsMatrix4(),
                              X_WS_.GetAsMatrix4(), 1e-15));
  EXPECT_EQ(engine1_->num_color_renders(), 1);
  EXPECT_EQ(engine1_->num_depth_renders(), 0);
  EXPECT_EQ(engine1_->num_label_renders(), 0);

  // Passing another *valid* name doesn't throw (it finds the *other* engine).
  // But passing an invalid name throws.
  EXPECT_NO_THROW(geometry_state_.RenderColorImage(color_camera("engine2"),
                                                   parent_id_, X_PC_, &image));
  EXPECT_THROW(geometry_state_.RenderColorImage(color_camera("not_an_engine"),
                                                parent_id_, X_PC_, &image),
               std::exception);
}

TEST_F(GeometryStateRenderTest, RenderDepthImage) {
  // The image doesn't matter; it will *not* get written to. It just needs to be
  // compatible with the camera.
  systems::sensors::ImageDepth32F image(width(), height());

  // Case: valid render sets the viewport and calls the right API. We don't test
  //  to confirm that the image is passed along, that would be screamingly
  //  apparent.
  geometry_state_.RenderDepthImage(depth_camera("engine1"), parent_id_, X_PC_,
                                   &image);
  EXPECT_TRUE(CompareMatrices(engine1_->last_updated_X_WC().GetAsMatrix4(),
                              X_WS_.GetAsMatrix4(), 1e-15));
  EXPECT_EQ(engine1_->num_color_renders(), 0);
  EXPECT_EQ(engine1_->num_depth_renders(), 1);
  EXPECT_EQ(engine1_->num_label_renders(), 0);

  // Passing another *valid* name doesn't throw (it finds the *other* engine).
  // But passing an invalid name throws.
  EXPECT_NO_THROW(geometry_state_.RenderDepthImage(depth_camera("engine2"),
                                                   parent_id_, X_PC_, &image));
  EXPECT_THROW(geometry_state_.RenderDepthImage(depth_camera("not_an_engine"),
                                                parent_id_, X_PC_, &image),
               std::exception);
}

TEST_F(GeometryStateRenderTest, RenderLabelImage) {
  // The image doesn't matter; it will *not* get written to. It just needs to be
  // compatible with the camera.
  systems::sensors::ImageLabel16I image(width(), height());

  // Case: valid render sets the viewport and calls the right API. We don't test
  //  to confirm that the image is passed along, that would be screamingly
  //  apparent.
  geometry_state_.RenderLabelImage(color_camera("engine1"), parent_id_, X_PC_,
                                   &image);
  EXPECT_TRUE(CompareMatrices(engine1_->last_updated_X_WC().GetAsMatrix4(),
                              X_WS_.GetAsMatrix4(), 1e-15));
  EXPECT_EQ(engine1_->num_color_renders(), 0);
  EXPECT_EQ(engine1_->num_depth_renders(), 0);
  EXPECT_EQ(engine1_->num_label_renders(), 1);

  // Passing another *valid* name doesn't throw (it finds the *other* engine).
  // But passing an invalid name throws.
  EXPECT_NO_THROW(geometry_state_.RenderLabelImage(color_camera("engine2"),
                                                   parent_id_, X_PC_, &image));
  EXPECT_THROW(geometry_state_.RenderLabelImage(color_camera("not_an_engine"),
                                                parent_id_, X_PC_, &image),
               std::exception);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
