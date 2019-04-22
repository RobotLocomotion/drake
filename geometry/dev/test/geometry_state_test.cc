#include "drake/geometry/dev/geometry_state.h"

#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/dev/geometry_roles.h"
#include "drake/geometry/dev/internal_frame.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_set.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace dev {

using std::vector;

// Implementation of friend class that allows me to peek into the geometry state
// to confirm invariants on the state's internal workings as a result of
// operations.

template <class T>
class GeometryStateTester {
 public:
  void set_state(GeometryState<T>* state) { state_ = state; }

  FrameId get_world_frame() const {
    return internal::InternalFrame::world_frame_id();
  }

  const std::unordered_map<SourceId, std::string>& get_source_name_map() const {
    return state_->source_names_;
  }

  const std::unordered_map<SourceId, FrameIdSet>& get_source_frame_id_map()
      const {
    return state_->source_frame_id_map_;
  }

  const std::unordered_map<SourceId, FrameIdSet>& get_source_root_frame_map()
      const {
    return state_->source_root_frame_map_;
  }

  const std::unordered_map<SourceId, std::unordered_set<GeometryId>>&
  get_source_anchored_geometry_map() const {
    return state_->source_anchored_geometry_map_;
  }

  const std::unordered_map<FrameId, internal::InternalFrame>& get_frames()
      const {
    return state_->frames_;
  }

  const std::unordered_map<GeometryId, internal::InternalGeometry>&
  get_geometries() const {
    return state_->geometries_;
  }

  const vector<GeometryId>& get_geometry_index_id_map() const {
    return state_->geometry_index_id_map_;
  }

  const vector<FrameId>& get_pose_index_frame_id_map() const {
    return state_->frame_index_to_frame_map_;
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

  void SetFramePoses(const FramePoseVector<T>& poses) {
    state_->SetFramePoses(poses);
  }

  // Returns the internal index for the geometry with the given index; if there
  // is a geometry that *has* that render index.
  optional<InternalIndex> InternalIndexFromRenderIndex(
      RenderIndex index) const {
    if (state_->X_WG_perception_.count(index) > 0) {
      // It's dynamic and we have a mapping.
      InternalIndex internal_index = state_->X_WG_perception_[index];
      // Confirm that it is internally consistent.
      EXPECT_EQ(
          state_->geometries_[state_->geometry_index_id_map_[internal_index]]
              .render_index(),
          index);
      return internal_index;
    } else {
      // It's apparently anchored geometry.
      for (auto& pair : state_->geometries_) {
        if (pair.second.has_perception_role() &&
            pair.second.render_index() == index) {
          return pair.second.internal_index();
        }
      }
    }
    return {};
  }

  void FinalizePoseUpdate() {
    state_->FinalizePoseUpdate();
  }

  template <typename ValueType>
  void ValidateFrameIds(const FrameKinematicsVector<ValueType>& data) const {
    state_->ValidateFrameIds(data);
  }

  int peek_next_clique() const {
    return state_->peek_next_clique();
  }

  const internal::InternalGeometry* GetGeometry(GeometryId id) {
    return state_->GetGeometry(id);
  }

 private:
  GeometryState<T>* state_;
};

namespace {

using Eigen::Isometry3d;
using internal::InternalFrame;
using internal::InternalGeometry;
using std::make_unique;
using std::move;
using std::unique_ptr;

// A simple dummy render engine implementation to facilitate testing. The
// methods are mostly no-ops. The single exception is in registering geometry.
// Every call returns a valid RenderIndex with the value `n` for the `n`th
// call to `RegisterVisual()`.
class DummyRenderEngine final : public render::RenderEngine {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DummyRenderEngine);
  DummyRenderEngine() = default;
  void UpdateViewpoint(const Eigen::Isometry3d&) const final {}
  void RenderColorImage(const render::CameraProperties&,
                        systems::sensors::ImageRgba8U*, bool) const final {}
  void RenderDepthImage(const render::DepthCameraProperties&,
                        systems::sensors::ImageDepth32F*) const final {}
  void RenderLabelImage(const render::CameraProperties&,
                        systems::sensors::ImageLabel16I*, bool) const final {}
  void ImplementGeometry(const Sphere& sphere, void* user_data) final {}
  void ImplementGeometry(const Cylinder& cylinder, void* user_data) final {}
  void ImplementGeometry(const HalfSpace& half_space, void* user_data) final {}
  void ImplementGeometry(const Box& box, void* user_data) final {}
  void ImplementGeometry(const Mesh& mesh, void* user_data) final {}
  void ImplementGeometry(const Convex& convex, void* user_data) final {}

  void set_moved_render_index(optional<RenderIndex> index) {
    moved_render_index_ = index;
  }

 protected:
  optional<RenderIndex> DoRegisterVisual(const Shape&,
                                         const PerceptionProperties&,
                                         const Isometry3<double>&) final {
    return RenderIndex(calls_to_register_++);
  }
  void DoUpdateVisualPose(const Eigen::Isometry3d&, RenderIndex) final {}

  optional<RenderIndex> DoRemoveGeometry(RenderIndex index) final {
    return moved_render_index_;
  }

  unique_ptr<render::RenderEngine> DoClone() const final {
    return make_unique<DummyRenderEngine>(*this);
  }

 private:
  int calls_to_register_{};
  // The value that `DoRemoveGeometry()` returns. Configurable by test. Defaults
  // to returning nothing.
  optional<RenderIndex> moved_render_index_{nullopt};
};

class GeometryStateTest : public ::testing::Test {
 protected:
  void SetUp() {
    frame_ = make_unique<GeometryFrame>("ref_frame");
    instance_pose_.translation() << 10, 20, 30;
    instance_ = make_unique<GeometryInstance>(
        instance_pose_, make_unique<Sphere>(1.0), "instance");
    gs_tester_.set_state(&geometry_state_);
    auto render_engine = make_unique<DummyRenderEngine>();
    render_engine_ = render_engine.get();
    geometry_state_.AddRenderer(dummy_render_name_, move(render_engine));
  }

  // Utility method for adding a source to the state.
  SourceId NewSource(const std::string& name = "") {
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
  SourceId SetUpSingleSourceTree(bool assign_proximity_role = false) {
    using std::to_string;

    source_id_ = NewSource();

    // Create f0.
    Isometry3<double> pose = Isometry3<double>::Identity();
    pose.translation() << 1, 2, 3;
    pose.linear() << 1, 0, 0, 0, 0, 1, 0, -1, 0;  // 90° around x-axis.
    frames_.push_back(geometry_state_.RegisterFrame(
        source_id_, GeometryFrame("f0")));
    X_WF_.push_back(pose);
    X_PF_.push_back(pose);

    // Create f1.
    pose.translation() << 10, 20, 30;
    pose.linear() << 0, 0, -1, 0, 1, 0, 1, 0, 0;  // 90° around y-axis.
    frames_.push_back(geometry_state_.RegisterFrame(
        source_id_, GeometryFrame("f1")));
    X_WF_.push_back(pose);
    X_PF_.push_back(pose);

    // Create f2.
    pose = pose.inverse();
    frames_.push_back(geometry_state_.RegisterFrame(
        source_id_, frames_[1], GeometryFrame("f2")));
    X_WF_.push_back(X_WF_[1] * pose);
    X_PF_.push_back(pose);

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
        const std::string& name =
            to_string(frame_id) + "_g" + std::to_string(i);
        geometry_names_[g_count] = name;
        geometries_[g_count] = geometry_state_.RegisterGeometry(
            source_id_, frame_id,
            make_unique<GeometryInstance>(pose, make_unique<Sphere>(1), name));
        if (assign_proximity_role) {
          geometry_state_.AssignRole(source_id_, geometries_[g_count],
                                     ProximityProperties());
        }
        X_FG_.push_back(pose);
        ++g_count;
      }
    }

    // Create anchored geometry.
    pose = Isometry3<double>::Identity();
    pose.translation() << 0, 0, -1;
    // This simultaneously tests the ability to register an anchored geometry by
    // explicitly calling out the world frame id and, indirectly, the
    // RegisterAnchoredGeometry() (which gets invoked in this case).
    anchored_geometry_ = geometry_state_.RegisterGeometry(
        source_id_, InternalFrame::world_frame_id(),
        make_unique<GeometryInstance>(
            pose, make_unique<Box>(100, 100, 2), anchored_name_));
    if (assign_proximity_role) {
      geometry_state_.AssignRole(source_id_, anchored_geometry_,
                                 ProximityProperties());
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

  int default_collision_pair_count() const {
    // Without filtering, this should be the expected pairs:
    // (a, g4), (a, g5)
    return 2;
  }

  // This method confirms that the stored dummy identifiers don't map to any
  // registered source identifier. This should only be invoked for scenarios
  // where there is *only* the single source.
  void AssertSingleTreeCleared() {
    // Confirms frames have been cleared.
    for (int f = 0; f < kFrameCount; ++f) {
      DRAKE_EXPECT_THROWS_MESSAGE(
          geometry_state_.BelongsToSource(frames_[f], source_id_),
          std::logic_error, "Referenced frame \\d+ has not been registered.");
    }
    // Confirms geometries have been cleared.
    for (int g = 0; g < kFrameCount * kGeometryCount; ++g) {
      DRAKE_EXPECT_THROWS_MESSAGE(
          geometry_state_.BelongsToSource(geometries_[g], source_id_),
          std::logic_error,
          "Referenced geometry \\d+ has not been registered.");
    }
    EXPECT_EQ(gs_tester_.get_source_frame_id_map().at(source_id_).size(), 0);
    EXPECT_EQ(gs_tester_.get_source_frame_id_map().size(), 1);
    EXPECT_EQ(gs_tester_.get_source_root_frame_map().at(source_id_).size(), 0);
    EXPECT_EQ(gs_tester_.get_source_root_frame_map().size(), 1);
    EXPECT_EQ(gs_tester_.get_frames().size(), 0);
    EXPECT_EQ(gs_tester_.get_geometries().size(), 0);
    EXPECT_EQ(gs_tester_.get_frame_parent_poses().size(), 0);
    EXPECT_EQ(gs_tester_.get_geometry_world_poses().size(), 0);
  }

  // Utility function for facilitating tests; confirms that the identified
  // frame doesn't belong to the identified source. This explicitly tests the
  // underlying state data structures.
  void ExpectSourceDoesNotHaveFrame(SourceId source_id, FrameId frame_id) {
    // Frame is not in the source-to-set-of-frame-ids mapping.
    EXPECT_EQ(gs_tester_.get_source_frame_id_map().at(source_id).find(frame_id),
              gs_tester_.get_source_frame_id_map().at(source_id).end());
    // Frame is not in the source-to-set-of-root-ids mapping.
    EXPECT_EQ(
        gs_tester_.get_source_root_frame_map().at(source_id).find(frame_id),
        gs_tester_.get_source_root_frame_map().at(source_id).end());
    // Frame not in frames
    EXPECT_EQ(gs_tester_.get_frames().find(frame_id),
              gs_tester_.get_frames().end());
  }

  // Members owned by the test class.
  unique_ptr<GeometryFrame> frame_;
  unique_ptr<GeometryInstance> instance_;
  Isometry3<double> instance_pose_{Isometry3<double>::Identity()};
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
  // The geometry ids created in the dummy tree instantiation.
  vector<GeometryId> geometries_;
  // The names for all the geometries (as registered).
  vector<std::string> geometry_names_;
  // The single, anchored geometry id.
  GeometryId anchored_geometry_;
  // The registered name of the anchored geometry.
  const std::string anchored_name_{"anchored"};
  // The id of the single-source tree.
  SourceId source_id_;

  // The poses of the frames in the world frame.
  vector<Isometry3<double>> X_WF_;
  // The poses of the frames in the parent's frame.
  vector<Isometry3<double>> X_PF_;
  // The poses of the geometries in the parent frame.
  vector<Isometry3<double>> X_FG_;
  // The default source name.
  const std::string kSourceName{"default_source"};
  // The name of the dummy renderer added to the geometry state.
  const std::string dummy_render_name_{"dummy_renderer"};
};

// Confirms that a new GeometryState has no data.
TEST_F(GeometryStateTest, Constructor) {
  // GeometryState has a "self source".
  EXPECT_EQ(geometry_state_.get_num_sources(), 1);
  // GeometryState always has a world frame.
  EXPECT_EQ(geometry_state_.get_num_frames(), 1);
  EXPECT_EQ(geometry_state_.get_num_geometries(), 0);
}

// Confirms semantics of user-specified source name.
//    - The source name is stored and retrievable,
//    - duplicate names are detected and considered errors, and
//    - unrecognized source ids do not produce names.
TEST_F(GeometryStateTest, SourceRegistrationWithNames) {
  using std::to_string;

  // Case: Successful registration of unique source id and name.
  SourceId s_id;
  std::string name = "Unique";
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
  SourceId dummy_source = SetUpSingleSourceTree();
  EXPECT_TRUE(geometry_state_.source_is_registered(dummy_source));
  // Dummy source + self source.
  EXPECT_EQ(geometry_state_.get_num_sources(), 2);
  EXPECT_EQ(geometry_state_.get_num_frames(), single_tree_frame_count());
  EXPECT_EQ(geometry_state_.GetNumDynamicGeometries(),
            single_tree_dynamic_geometry_count());
  EXPECT_EQ(geometry_state_.GetNumAnchoredGeometries(),
            anchored_geometry_count());
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            single_tree_total_geometry_count());
  SourceId false_id = SourceId::get_new_id();
  EXPECT_FALSE(geometry_state_.source_is_registered(false_id));
}

// Compares the autodiff geometry state (embedded in its tester) against the
// double state to confirm they have the same values/topology.
void ExpectSuccessfulTransmogrification(
    const GeometryStateTester<AutoDiffXd>& ad_tester,
    const GeometryStateTester<double>& d_tester) {

  // 1. Test all of the identifier -> trivially testable value maps
  EXPECT_EQ(ad_tester.get_source_name_map(), d_tester.get_source_name_map());
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
  EXPECT_EQ(ad_tester.get_geometry_index_id_map(),
            d_tester.get_geometry_index_id_map());
  EXPECT_EQ(ad_tester.get_pose_index_frame_id_map(),
            d_tester.get_pose_index_frame_id_map());

  // 3. Compare Isometry3<double> with Isometry3<double>
  for (GeometryId id : ad_tester.get_geometry_index_id_map()) {
    EXPECT_TRUE(CompareMatrices(
        ad_tester.get_geometries().at(id).X_FG().matrix().block<3, 4>(0, 0),
        d_tester.get_geometries().at(id).X_FG().matrix().block<3, 4>(0, 0)));
  }

  // 4. Compare Isometry3<AutoDiffXd> with Isometry3<double>
  auto test_ad_vs_double = [](const std::vector<Isometry3<AutoDiffXd>>& test,
                              const std::vector<Isometry3<double>>& ref) {
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
  SourceId s_id = SetUpSingleSourceTree();

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
    auto test_frame = [internal_frames, this, s_id](int i, FrameId parent_id,
                                                    int num_child_frames) {
      const auto& frame = internal_frames.at(frames_[i]);
      EXPECT_EQ(frame.source_id(), s_id);
      EXPECT_EQ(frame.id(), frames_[i]);
      EXPECT_EQ(frame.name(), "f" + to_string(i));
      EXPECT_EQ(frame.frame_group(), 0);  // Defaults to zero.
      EXPECT_EQ(frame.internal_index(), i + 1);   // ith frame added.
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
          CompareMatrices(frame_in_parent[frame.internal_index()].matrix(),
                          X_PF_[i].matrix()));
    };

    // When added, all frames' poses w.r.t. their parents are the identity.
    const auto& frame_in_parent = gs_tester_.get_frame_parent_poses();
    for (FrameId frame_id : frames_) {
      const auto& frame = internal_frames.at(frame_id);
      EXPECT_TRUE(
          CompareMatrices(frame_in_parent[frame.internal_index()].matrix(),
                          Isometry3<double>::Identity().matrix()));
    }

    // Confirm posing positions the frames properly.
    FramePoseVector<double> poses(source_id_, frames_);
    poses.clear();
    for (int f = 0; f < static_cast<int>(frames_.size()); ++f) {
      poses.set_value(frames_[f], X_PF_[f]);
    }
    gs_tester_.SetFramePoses(poses);
    gs_tester_.FinalizePoseUpdate();

    test_frame(0, gs_tester_.get_world_frame(), 0);
    test_frame(1, gs_tester_.get_world_frame(), 1);
    test_frame(2, frames_[1], 0);
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
      EXPECT_EQ(geometry.internal_index(), i);
      // We automatically assign render labels to any geometry with non-zero
      // alpha value in its diffuse color.
      EXPECT_TRUE(geometry.render_index(dummy_render_name_));
      EXPECT_FALSE(geometry.proximity_index().is_valid());
      EXPECT_EQ(geometry.child_geometry_ids().size(), 0);

      // Note: There are no geometries parented to other geometries. The results
      // of GetPoseInFrame() and GetPoseInParent() must be the identical (as
      // the documentation for GeometryState::GetPoseInParent() indicates).
      EXPECT_TRUE(CompareMatrices(
          geometry_state_.GetPoseInFrame(geometry.id()).matrix(),
          X_FG_[i].matrix()));
      EXPECT_TRUE(CompareMatrices(
          geometry_state_.GetPoseInParent(geometry.id()).matrix(),
          X_FG_[i].matrix()));

      EXPECT_EQ(
          gs_tester_.get_geometry_index_id_map()[geometry.internal_index()],
          geometry.id());
    }
  }
  EXPECT_EQ(static_cast<int>(gs_tester_.get_geometry_world_poses().size()),
            single_tree_total_geometry_count());
  EXPECT_EQ(gs_tester_.get_frame_parent_poses().size(), kFrameCount + 1);
}

// Tests the GetNum*Geometry*Methods.
TEST_F(GeometryStateTest, GetNumGeometryTests) {
  SetUpSingleSourceTree(true /* add proximity roles */);

  EXPECT_EQ(single_tree_total_geometry_count(),
            geometry_state_.get_num_geometries());
  EXPECT_EQ(single_tree_total_geometry_count(),
            geometry_state_.GetNumGeometriesWithRole(Role::kProximity));
  EXPECT_EQ(single_tree_total_geometry_count(),
            geometry_state_.GetNumGeometriesWithRole(Role::kPerception));
  EXPECT_EQ(single_tree_total_geometry_count(),
            geometry_state_.GetNumGeometriesWithRole(Role::kIllustration));

  for (int i = 0; i < kFrameCount; ++i) {
    EXPECT_EQ(kGeometryCount,
              geometry_state_.GetNumFrameGeometries(frames_[i]));
    EXPECT_EQ(kGeometryCount,
              geometry_state_.GetNumFrameGeometriesWithRole(frames_[i],
                                                            Role::kProximity));
    EXPECT_EQ(kGeometryCount,
              geometry_state_.GetNumFrameGeometriesWithRole(
                  frames_[i], Role::kPerception));
    EXPECT_EQ(kGeometryCount,
              geometry_state_.GetNumFrameGeometriesWithRole(
                  frames_[i], Role::kIllustration));
  }
}

// Tests that an attempt to add a frame to an invalid source throws an exception
// with meaningful message.
TEST_F(GeometryStateTest, AddFrameToInvalidSource) {
  SourceId s_id = SourceId::get_new_id();  // This is not a registered source.
  DRAKE_ASSERT_THROWS_MESSAGE(
      geometry_state_.RegisterFrame(s_id, *frame_.get()), std::logic_error,
      "Referenced geometry source \\d+ is not registered.");
}

// Tests that a frame added to a valid source appears in the source's frames.
TEST_F(GeometryStateTest, AddFirstFrameToValidSource) {
  SourceId s_id = NewSource();
  FrameId fid = geometry_state_.RegisterFrame(s_id, *frame_.get());
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
  SourceId s_id = SetUpSingleSourceTree();
  FrameId fid = geometry_state_.RegisterFrame(s_id, *frame_);
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
  SourceId s_id = SetUpSingleSourceTree();
  SourceId new_s_id = geometry_state_.RegisterNewSource("new_source");
  FrameId fid = geometry_state_.RegisterFrame(new_s_id, *frame_.get());
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
  SourceId s_id = SetUpSingleSourceTree();
  FrameId fid = geometry_state_.RegisterFrame(s_id, frames_[0], *frame_);
  EXPECT_EQ(fid, frame_->id());
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
  SourceId s_id = NewSource();
  FrameId f_id = geometry_state_.RegisterFrame(s_id, *frame_.get());
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
  std::unordered_set<FrameId> all_frames(frames_.begin(), frames_.end());
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
  SourceId s_id = NewSource();
  FrameId f_id = geometry_state_.RegisterFrame(s_id, *frame_);
  GeometryId expected_g_id = instance_->id();
  GeometryId g_id = geometry_state_.RegisterGeometry(s_id, f_id,
                                                     move(instance_));
  EXPECT_EQ(g_id, expected_g_id);
  EXPECT_EQ(geometry_state_.GetFrameId(g_id), f_id);
  EXPECT_TRUE(geometry_state_.BelongsToSource(g_id, s_id));
  Isometry3<double> X_FG = geometry_state_.GetPoseInFrame(g_id);
  EXPECT_TRUE(CompareMatrices(X_FG.matrix(), instance_pose_.matrix()));

  EXPECT_TRUE(gs_tester_.get_frames().at(f_id).has_child(g_id));
  const auto& geometry = gs_tester_.get_geometries().at(g_id);
  EXPECT_TRUE(geometry.is_child_of_frame(f_id));
  EXPECT_FALSE(geometry.parent_id());
}

// Confirms that registering two geometries with the same id causes failure.
TEST_F(GeometryStateTest, RegisterDuplicateGeometry) {
  SourceId s_id = NewSource();
  FrameId f_id = geometry_state_.RegisterFrame(s_id, *frame_);
  auto instance_copy = make_unique<GeometryInstance>(*instance_);
  geometry_state_.RegisterGeometry(s_id, f_id, move(instance_));
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterGeometry(s_id, f_id, move(instance_copy)),
      std::logic_error,
      "Registering geometry with an id that has already been registered: \\d+");
}

// Tests registration of geometry on invalid source.
TEST_F(GeometryStateTest, RegisterGeometryMissingSource) {
  SourceId s_id = SourceId::get_new_id();
  FrameId f_id = FrameId::get_new_id();
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterGeometry(s_id, f_id, move(instance_)),
      std::logic_error, "Referenced geometry source \\d+ is not registered.");
}

// Tests registration of geometry on valid source and non-existent frame.
TEST_F(GeometryStateTest, RegisterGeometryMissingFrame) {
  SourceId s_id = NewSource();

  FrameId f_id = FrameId::get_new_id();
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterGeometry(s_id, f_id, move(instance_)),
      std::logic_error,
      "Referenced frame \\d+ for source \\d+\\,"
      " but the frame doesn't belong to the source.");
}

// Tests error resulting from passing a null GeometryInstance.
TEST_F(GeometryStateTest, RegisterNullGeometry) {
  SourceId s_id = NewSource();
  FrameId f_id = geometry_state_.RegisterFrame(s_id, *frame_);
  unique_ptr<GeometryInstance> null_geometry;
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterGeometry(s_id, f_id, move(null_geometry)),
      std::logic_error,
      "Registering null geometry to frame \\d+, on source \\d+.");
}

// Tests the logic for hanging a geometry on another geometry. This confirms
// topology and pose values.
TEST_F(GeometryStateTest, RegisterGeometryonValidGeometry) {
  SourceId s_id = SetUpSingleSourceTree();
  const double x = 3;
  const double y = 2;
  const double z = 1;
  Isometry3<double> pose = Isometry3<double>::Identity();
  pose.translation() << x, y, z;
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
  Isometry3<double> expected_pose_in_frame = Isometry3<double>::Identity();
  expected_pose_in_frame.translation() << (parent_index + 1) + x, y, z;
  EXPECT_EQ(frame_id, geometry_state_.GetFrameId(g_id));

  Isometry3<double> X_FG = geometry_state_.GetPoseInFrame(g_id);
  EXPECT_TRUE(CompareMatrices(X_FG.matrix(), expected_pose_in_frame.matrix(),
                  1e-14, MatrixCompareType::absolute));
  Isometry3<double> X_PG = geometry_state_.GetPoseInParent(g_id);
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
  SourceId s_id = SetUpSingleSourceTree();
  Isometry3<double> pose = Isometry3<double>::Identity();
  auto instance = make_unique<GeometryInstance>(
      pose, make_unique<Sphere>(1), "sphere");
  GeometryId junk_id = GeometryId::get_new_id();
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterGeometryWithParent(s_id, junk_id, move(instance)),
      std::logic_error,
      "Referenced geometry \\d+ has not been registered.");
}

// Tests the response to passing a null pointer as a GeometryInstance.
TEST_F(GeometryStateTest, RegisterNullGeometryonGeometry) {
  SourceId s_id = SetUpSingleSourceTree();
  unique_ptr<GeometryInstance> instance;
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterGeometryWithParent(s_id, geometries_[0],
                                                 move(instance)),
      std::logic_error,
      "Registering null geometry to geometry \\d+, on source \\d+.");
}

// Tests the registration of anchored geometry.
TEST_F(GeometryStateTest, RegisterAnchoredGeometry) {
  SourceId s_id = NewSource("new source");
  Isometry3<double> pose = Isometry3<double>::Identity();
  auto instance = make_unique<GeometryInstance>(
      pose, make_unique<Sphere>(1), "sphere");
  GeometryId expected_g_id = instance->id();
  auto g_id = geometry_state_.RegisterAnchoredGeometry(s_id, move(instance));
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
  SourceId s_id = NewSource("new source");
  Isometry3<double> pose = Isometry3<double>::Identity();
  pose.translation() << 1, 2, 3;
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
  EXPECT_TRUE(CompareMatrices((parent->X_FG() * pose).matrix(),
                              child->X_FG().matrix()));
  EXPECT_EQ(InternalFrame::world_frame_id(), parent->frame_id());
}

// Confirms that registering two geometries with the same id causes failure.
TEST_F(GeometryStateTest, RegisterDuplicateAnchoredGeometry) {
  SourceId s_id = NewSource();
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
  Isometry3<double> pose = Isometry3<double>::Identity();
  auto instance = make_unique<GeometryInstance>(
      pose, make_unique<Sphere>(1), "sphere");
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
  SourceId source_id = SourceId::get_new_id();
  // Invalid frame/geometry ids.
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.BelongsToSource(FrameId::get_new_id(), source_id),
      std::logic_error, "Referenced geometry source \\d+ is not registered.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.BelongsToSource(GeometryId::get_new_id(), source_id),
      std::logic_error, "Referenced geometry source \\d+ is not registered.");
  SetUpSingleSourceTree();
  GeometryId anchored_id = geometry_state_.RegisterAnchoredGeometry(
      source_id_,
      make_unique<GeometryInstance>(Isometry3<double>::Identity(),
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
  SourceId s_id = SetUpSingleSourceTree();
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
  SourceId s_id = SetUpSingleSourceTree();
  GeometryId anchored_id = geometry_state_.RegisterAnchoredGeometry(
      s_id, make_unique<GeometryInstance>(Isometry3<double>::Identity(),
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
  SourceId s_id = SetUpSingleSourceTree();
  FramePoseVector<double> frame_set(s_id, frames_);
  frame_set.clear();
  for (auto frame_id : frames_) {
    frame_set.set_value(frame_id, Isometry3<double>::Identity());
  }
  // Case: frame ids are valid.
  EXPECT_NO_THROW(gs_tester_.ValidateFrameIds(frame_set));

  // Case: Right number, wrong frames.
  vector<FrameId> bad_frames;
  for (int i = 0; i < kFrameCount; ++i) {
    bad_frames.push_back(FrameId::get_new_id());
  }
  FramePoseVector<double> frame_set_2(s_id, bad_frames);
  DRAKE_EXPECT_THROWS_MESSAGE(
      gs_tester_.ValidateFrameIds(frame_set_2), std::runtime_error,
      "Registered frame id \\(\\d+\\) belonging to source \\d+ was not found "
          "in the provided kinematics data.");

  // Case: Too few frames.
  vector<FrameId> missing_frames;
  for (int i = 0; i < kFrameCount - 1; ++i) {
    missing_frames.push_back(frames_[i]);
  }
  FramePoseVector<double> frame_set_3(s_id, missing_frames);
  DRAKE_EXPECT_THROWS_MESSAGE(
      gs_tester_.ValidateFrameIds(frame_set_3), std::runtime_error,
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
  SourceId s_id = SetUpSingleSourceTree();
  // A vector of poses we will use to populate FramePoseVectors.
  vector<Isometry3<double>> frame_poses;
  for (int i = 0; i < kFrameCount; ++i) {
    frame_poses.push_back(Isometry3<double>::Identity());
  }

  auto make_pose_vector =
      [&s_id, &frame_poses, this]() -> FramePoseVector<double> {
    const int count = static_cast<int>(this->frames_.size());
    FramePoseVector<double> poses(s_id, this->frames_);
    poses.clear();
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
  gs_tester_.SetFramePoses(poses1);
  const auto& world_poses = gs_tester_.get_geometry_world_poses();
  for (int i = 0; i < kFrameCount * kGeometryCount; ++i) {
    EXPECT_TRUE(CompareMatrices(world_poses[i].matrix().block<3, 4>(0, 0),
                                X_FG_[i].matrix().block<3, 4>(0, 0)));
  }

  // Case 2: Move the two *root* frames 1 unit in the +y direction. The f2 will
  // stay at the identity.
  // The final geometry poses should all be offset by 1 unit in the y.
  Isometry3<double> offset = Isometry3<double>::Identity();
  offset.translation() << 0, 1, 0;
  frame_poses[0] = offset;
  frame_poses[1] = offset;
  FramePoseVector<double> poses2 = make_pose_vector();
  gs_tester_.SetFramePoses(poses2);
  for (int i = 0; i < kFrameCount * kGeometryCount; ++i) {
    EXPECT_TRUE(
        CompareMatrices(world_poses[i].matrix().block<3, 4>(0, 0),
                        (offset * X_FG_[i].matrix()).block<3, 4>(0, 0)));
  }

  // Case 3: All frames get set to move up one unit. This will leave geometries
  // 0, 1, 2, & 3 moved up 1, and geometries 4 & 5 moved up two.
  frame_poses[2] = offset;
  FramePoseVector<double> poses3 = make_pose_vector();
  gs_tester_.SetFramePoses(poses3);
  for (int i = 0; i < (kFrameCount - 1) * kGeometryCount; ++i) {
    EXPECT_TRUE(
        CompareMatrices(world_poses[i].matrix().block<3, 4>(0, 0),
                        (offset * X_FG_[i].matrix()).block<3, 4>(0, 0)));
  }
  for (int i = (kFrameCount - 1) * kGeometryCount;
       i < kFrameCount * kGeometryCount; ++i) {
    EXPECT_TRUE(CompareMatrices(
        world_poses[i].matrix().block<3, 4>(0, 0),
        (offset * offset * X_FG_[i].matrix()).block<3, 4>(0, 0)));
  }
}

// Test various frame property queries.
TEST_F(GeometryStateTest, QueryFrameProperties) {
  SourceId s_id = SetUpSingleSourceTree();
  const FrameId world = InternalFrame::world_frame_id();

  // Query frame group.
  EXPECT_EQ(geometry_state_.get_frame_group(frames_[0]), 0);
  EXPECT_EQ(geometry_state_.get_frame_group(world), 1234567);
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
  FramePoseVector<double> poses(s_id, frames_);
  poses.clear();
  for (int i = 0; i < kFrameCount; ++i) poses.set_value(frames_[i], X_PF_[i]);
  gs_tester_.SetFramePoses(poses);

  EXPECT_TRUE(
      CompareMatrices(geometry_state_.get_pose_in_world(frames_[0]).matrix(),
                      X_WF_[0].matrix()));
  EXPECT_TRUE(
      CompareMatrices(geometry_state_.get_pose_in_world(world).matrix(),
                      Isometry3d::Identity().matrix()));
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.get_pose_in_world(FrameId::get_new_id()),
      std::logic_error, "No world pose available for invalid frame id: \\d+");

  // This assumes that geometry parent belongs to frame 0.
  Isometry3<double> geometry_pose = X_WF_[0] * X_FG_[0];
  EXPECT_TRUE(CompareMatrices(
      geometry_state_.get_pose_in_world(geometries_[0]).matrix(),
      geometry_pose.matrix()));
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.get_pose_in_world(GeometryId::get_new_id()),
      std::logic_error,
      "No world pose available for invalid geometry id: \\d+");

  EXPECT_TRUE(
      CompareMatrices(geometry_state_.get_pose_in_parent(frames_[0]).matrix(),
                      X_PF_[0].matrix()));
  EXPECT_TRUE(
      CompareMatrices(geometry_state_.get_pose_in_parent(world).matrix(),
                      Isometry3d::Identity().matrix()));
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.get_pose_in_parent(FrameId::get_new_id()),
      std::logic_error, "No pose available for invalid frame id: \\d+");
}

// Test disallowing collisions among members of a group (self collisions).
TEST_F(GeometryStateTest, ExcludeCollisionsWithin) {
  SetUpSingleSourceTree(true /* assign proximity roles */);

  // Pose all of the frames to the specified poses in their parent frame.
  FramePoseVector<double> poses(source_id_, frames_);
  poses.clear();
  for (int f = 0; f < static_cast<int>(frames_.size()); ++f) {
    poses.set_value(frames_[f], X_PF_[f]);
  }
  gs_tester_.SetFramePoses(poses);
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
  geometry_state_.ExcludeCollisionsWithin(
      GeometrySet({anchored_geometry_}, {frames_[0], frames_[1]}));
  pairs = geometry_state_.ComputePointPairPenetration();
  ASSERT_EQ(static_cast<int>(pairs.size()), expected_collisions);

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
  SetUpSingleSourceTree(true  /* add proximity roles */);

  // Pose all of the frames to the specified poses in their parent frame.
  FramePoseVector<double> poses(source_id_, frames_);
  poses.clear();
  for (int f = 0; f < static_cast<int>(frames_.size()); ++f) {
    poses.set_value(frames_[f], X_PF_[f]);
  }
  gs_tester_.SetFramePoses(poses);
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
  // Default tree *all* has proximity queries.
  SetUpSingleSourceTree(true  /* add proximity roles */);

  // Pose all of the frames to the specified poses in their parent frame.
  FramePoseVector<double> poses(source_id_, frames_);
  poses.clear();
  for (int f = 0; f < static_cast<int>(frames_.size()); ++f) {
    poses.set_value(frames_[f], X_PF_[f]);
  }
  gs_tester_.SetFramePoses(poses);
  gs_tester_.FinalizePoseUpdate();

  // This is *non* const; we'll decrement it as we filter more and more
  // collisions.
  int expected_collisions = default_collision_pair_count();

  // Baseline collision - the unfiltered collisions.
  auto pairs = geometry_state_.ComputePointPairPenetration();
  EXPECT_EQ(static_cast<int>(pairs.size()), expected_collisions);

  // Now add a new collision element to the third frame that *should* put it
  // in contact with the anchored geometry -- if it had a proximity role.
  Isometry3<double> pose = Isometry3<double>::Identity();
  // Place the new sphere so that it would collide with both of the current
  // spheres -- this confirms that adding the proximity role will still properly
  // include it in the collision filter for geometries under the same frame.
  pose.translation() << 5.5, 0, 0;
  // Have the name reflect the frame and the index in the geometry.
  const std::string& name("added_sphere");
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

  // If we assign a role, the collisions go up by one -- the previous attempt
  // to filter collisions had no affect and we increase the number of
  // collisions.
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
  GeometrySet set_bad_frame{FrameId::get_new_id(), FrameId::get_new_id()};
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.ExcludeCollisionsWithin(set_bad_frame), std::logic_error,
      "Referenced frame \\d+ has not been registered.");

  GeometrySet set_bad_geometry{GeometryId::get_new_id(),
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

  GeometrySet set_bad_frame{FrameId::get_new_id()};
  GeometrySet set_good_frame{frames_[0]};
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

  GeometrySet set_bad_geometry{GeometryId::get_new_id()};
  GeometrySet set_good_geometry{geometries_[0]};
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

// Tests the ability to query for a geometry from the name of a geometry.
TEST_F(GeometryStateTest, GetGeometryIdFromName) {
  SetUpSingleSourceTree(true /* initialize with proximity role */);
  // Frame i has geometries f * kFrameCount + g, where g ∈ [0, kGeometryCount).
  for (int f = 0; f < kFrameCount; ++f) {
    for (int g = 0; g < kGeometryCount; ++g) {
      int g_index = f * kGeometryCount + g;
      GeometryId expected_id = geometries_[g_index];
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
}

// Confirms that the name *stored* with the geometry is the trimmed version of
// the requested name.
TEST_F(GeometryStateTest, GeometryNameStorage) {
  SetUpSingleSourceTree();

  const Isometry3d X_FG = Isometry3d::Identity();
  const std::string name = "unique test name";

  // White space trimmed off and new string stored.
  {
    GeometryId id = geometry_state_.RegisterGeometry(
        source_id_, frames_[0],
        make_unique<GeometryInstance>(
            X_FG, make_unique<Sphere>(1), " " + name));
    EXPECT_EQ(geometry_state_.get_name(id), name);
  }

  // Valid name that is unchanged after trimming is stored as is.
  // Note: This assigns a geometry fo the *same* name to a *different* frame.
  {
    GeometryId id = geometry_state_.RegisterGeometry(
        source_id_, frames_[1],
        make_unique<GeometryInstance>(
            X_FG, make_unique<Sphere>(1), name));
    EXPECT_EQ(geometry_state_.get_name(id), name);
  }
}

// Tests the logic for confirming if a name is valid or not.
TEST_F(GeometryStateTest, GeometryNameValidation) {
  SetUpSingleSourceTree(true /* Assign proximity roles */);

  // Case: Invalid frame should throw (regardless of name contents or role).
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.IsValidGeometryName(FrameId::get_new_id(),
                                          Role::kProximity, ""),
      std::logic_error, "Given frame id is not valid: \\d+");

  auto expect_bad_name = [this](const std::string& name,
                                const std::string& exception_message,
                                const std::string& printable_name) {
    EXPECT_FALSE(geometry_state_.IsValidGeometryName(frames_[0],
                                                     Role::kProximity, name))
        << "Failed on input name: " << printable_name;
  };

  // Unique in world frame.
  const FrameId world = InternalFrame::world_frame_id();
  EXPECT_FALSE(geometry_state_.IsValidGeometryName(world, Role::kProximity,
                                                   anchored_name_));
  EXPECT_FALSE(geometry_state_.IsValidGeometryName(world, Role::kIllustration,
                                                   anchored_name_));
  EXPECT_TRUE(geometry_state_.IsValidGeometryName(world, Role::kProximity,
                                                  anchored_name_ + "2"));

  // Invalid cases:
  // Empty.
  expect_bad_name("", "The proposed geometry name is empty", "");

  // Nothing but whitespace.
  const std::string whitespace_message{
      "The proposed geometry name contains only whitespace"};
  expect_bad_name(" ", whitespace_message, "' '");
  expect_bad_name("\t", whitespace_message, "'\\t'");
  expect_bad_name(" \t", whitespace_message, "' \\t'");

  // Case: Valid (as a control case).
  const std::string unique = "unique";
  EXPECT_TRUE(geometry_state_.IsValidGeometryName(frames_[0], Role::kProximity,
                                                  unique));

  // Querying with non-canonical names test as the canonical name.
  vector<std::string> names{" " + unique, unique + " ", " " + unique + " "};
  for (const auto& name : names) {
    EXPECT_TRUE(geometry_state_.IsValidGeometryName(frames_[0],
                                                    Role::kProximity, name));
  }

  // Test potential duplicates.

  // A name with the same role will fail.
  EXPECT_FALSE(geometry_state_.IsValidGeometryName(
      frames_[0], Role::kProximity,
      gs_tester_.get_geometries().at(geometries_[0]).name()));

  // Case: Whitespace that SDF nevertheless considers not whitespace.
  // Update this when the following sdformat issue is resolved:
  // https://bitbucket.org/osrf/sdformat/issues/194/string-trimming-only-considers-space-and
  for (const std::string& s : {"\n", " \n\t", " \f", "\v", "\r", "\ntest"}) {
    EXPECT_TRUE(geometry_state_.IsValidGeometryName(frames_[0],
                                                    Role::kProximity, s));
  }
}

// Tests that the property values created get correctly propagated to the
// target geometry.
TEST_F(GeometryStateTest, RolePropertyValueAssignment) {
  SetUpSingleSourceTree();
  // Tests for proximity properties and assumes the same holds true for the
  // other role property types.

  ProximityProperties source;
  const std::string& default_group = source.default_group_name();
  source.AddProperty(default_group, "prop1", 7);
  source.AddProperty(default_group, "prop2", 10);
  const std::string group1("group1");
  source.AddGroup(group1);
  source.AddProperty(group1, "propA", 7.5);
  source.AddProperty(group1, "propB", "test");

  geometry_state_.AssignRole(source_id_, geometries_[0], source);
  const ProximityProperties* read =
      gs_tester_.GetGeometry(geometries_[0])->proximity_properties();
  ASSERT_NE(read, nullptr);

  // Test groups.
  ASSERT_EQ(source.num_groups(), read->num_groups());
  ASSERT_TRUE(read->has_group(group1));
  ASSERT_TRUE(read->has_group(default_group));

  // Test properties.
  EXPECT_EQ(source.NumProperties(), read->NumProperties());

  EXPECT_EQ(source.NumProperties(default_group),
            read->NumProperties(default_group));
  EXPECT_EQ(source.GetProperty<int>(default_group, "prop1"),
            read->GetProperty<int>(default_group, "prop1"));
  EXPECT_EQ(source.GetProperty<int>(default_group, "prop2"),
            read->GetProperty<int>(default_group, "prop2"));

  EXPECT_EQ(source.NumProperties(group1), read->NumProperties(group1));
  EXPECT_EQ(source.GetProperty<double>(group1, "propA"),
            read->GetProperty<double>(group1, "propA"));
  EXPECT_EQ(source.GetProperty<std::string>(group1, "propB"),
            read->GetProperty<std::string>(group1, "propB"));
}

// Tests the conditions in which `AssignRole()` throws an exception.
TEST_F(GeometryStateTest, RoleAssignExceptions) {
  SetUpSingleSourceTree();

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
  SourceId other_source = geometry_state_.RegisterNewSource("alt_source");
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

  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.AssignRole(source_id_, geometries_[0],
                                 PerceptionProperties()),
      std::logic_error,
      "Geometry already has perception role assigned");

  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.AssignRole(source_id_, geometries_[0],
                                 IllustrationProperties()),
      std::logic_error,
      "Geometry already has illustration role assigned");
}

// Tests that GeometryState's bookkeeping is consistent after removal of the
// illustration role from a _single_ geometry. This test is responsible for
// also covering the general conditions:
//   - removing unassigned or proximity roles.
TEST_F(GeometryStateTest, RemoveIllustrationFromGeometry) {
  SetUpSingleSourceTree();

  int expected_count = single_tree_total_geometry_count();

  // Relies on all geometries having illustration properties.
  EXPECT_EQ(geometry_state_.GetNumGeometriesWithRole(Role::kIllustration),
            expected_count);

  // Case: asking to remove unassigned has no affect.
  EXPECT_EQ(
      geometry_state_.RemoveRole(source_id_, geometries_[0], Role::kUnassigned),
      0);
  EXPECT_EQ(geometry_state_.GetNumGeometriesWithRole(Role::kIllustration),
            expected_count);

  // Case: asking to remove proximity has no affect.
  EXPECT_EQ(
      geometry_state_.RemoveRole(source_id_, geometries_[0], Role::kProximity),
      0);
  EXPECT_EQ(geometry_state_.GetNumGeometriesWithRole(Role::kIllustration),
            expected_count);

  // Case: removing illustration from a single geometry reports removal.
  const InternalGeometry* geometry = gs_tester_.GetGeometry(geometries_[0]);
  EXPECT_TRUE(geometry->has_illustration_role());
  EXPECT_EQ(geometry_state_.RemoveRole(source_id_, geometries_[0],
                                       Role::kIllustration),
            1);
  EXPECT_EQ(geometry_state_.GetNumGeometriesWithRole(Role::kIllustration),
            --expected_count);
  EXPECT_FALSE(geometry->has_illustration_role());

  // Case: attempting to remove illustration from a geometry that has none has
  // no effect.
  EXPECT_EQ(geometry_state_.RemoveRole(source_id_, geometries_[0],
                                       Role::kIllustration),
            0);
  EXPECT_EQ(geometry_state_.GetNumGeometriesWithRole(Role::kIllustration),
            expected_count);
  EXPECT_FALSE(geometry->has_illustration_role());
}

// Tests that GeometryState's bookkeeping is consistent after removal of the
// illustration role from a _frame_. This test is used to test the
// common functionality for removing _any_ role, i.e., handling the world frame.
TEST_F(GeometryStateTest, RemoveIllustrationFromFrame) {
  SetUpSingleSourceTree();

  // NOTE: Do not re-order these tests; they have an accumulatative effect.
  int expected_count = single_tree_total_geometry_count();

  // In each test, we confirm the number of geometries with illustration role
  // have advanced as expected but the total number of geometries haven't.

  // Relies on all geometries having illustration properties.
  EXPECT_EQ(geometry_state_.GetNumGeometriesWithRole(Role::kIllustration),
            expected_count);

  // Case: asking to remove unassigned has no affect.
  EXPECT_EQ(
      geometry_state_.RemoveRole(source_id_, frames_[0], Role::kUnassigned),
      0);
  EXPECT_EQ(geometry_state_.GetNumGeometriesWithRole(Role::kIllustration),
            expected_count);
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            single_tree_total_geometry_count());

  // Case: asking to remove proximity has no affect.
  EXPECT_EQ(
      geometry_state_.RemoveRole(source_id_, frames_[0], Role::kProximity),
      0);
  EXPECT_EQ(geometry_state_.GetNumGeometriesWithRole(Role::kIllustration),
            expected_count);
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            single_tree_total_geometry_count());

  // Case: removing illustration from the frame reports both geometries changed.
  const InternalGeometry* geometry1 = gs_tester_.GetGeometry(geometries_[0]);
  EXPECT_TRUE(geometry1->has_illustration_role());
  const InternalGeometry* geometry2 = gs_tester_.GetGeometry(geometries_[1]);
  EXPECT_TRUE(geometry2->has_illustration_role());
  EXPECT_EQ(geometry_state_.RemoveRole(source_id_, frames_[0],
                                       Role::kIllustration),
            2);
  expected_count -= 2;
  EXPECT_EQ(geometry_state_.GetNumGeometriesWithRole(Role::kIllustration),
            expected_count);
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            single_tree_total_geometry_count());
  EXPECT_FALSE(geometry1->has_illustration_role());
  EXPECT_FALSE(geometry2->has_illustration_role());

  // Case: attempting to remove illustration from the frame that has no
  // geometries with the role has no effect.
  EXPECT_EQ(geometry_state_.RemoveRole(source_id_, frames_[0],
                                       Role::kIllustration),
            0);
  EXPECT_EQ(geometry_state_.GetNumGeometriesWithRole(Role::kIllustration),
            expected_count);
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            single_tree_total_geometry_count());
  EXPECT_FALSE(geometry1->has_illustration_role());
  EXPECT_FALSE(geometry2->has_illustration_role());

  // Case: Remove from frame when one frame has the role and one frame does not.
  //    - Removes the role from one child geometry.
  geometry_state_.RemoveRole(source_id_, geometries_[2], Role::kIllustration);
  EXPECT_FALSE(gs_tester_.GetGeometry(geometries_[2])->has_illustration_role());
  EXPECT_EQ(geometry_state_.GetNumGeometriesWithRole(Role::kIllustration),
            --expected_count);
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            single_tree_total_geometry_count());

  //    - Invokes remove on the frame - only the single remaining geometry
  //      should be affected.
  EXPECT_TRUE(gs_tester_.GetGeometry(geometries_[3])->has_illustration_role());
  EXPECT_EQ(geometry_state_.RemoveRole(source_id_, frames_[1],
                                       Role::kIllustration),
            1);
  EXPECT_EQ(geometry_state_.GetNumGeometriesWithRole(Role::kIllustration),
            --expected_count);
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            single_tree_total_geometry_count());
  EXPECT_FALSE(gs_tester_.GetGeometry(geometries_[3])->has_illustration_role());

  // Case: Operate on the world frame with a source that has no anchored
  // geometry. Should change nothing with no complaints.
  SourceId source_id_2 = geometry_state_.RegisterNewSource("source2");
  EXPECT_EQ(geometry_state_.RemoveRole(
      source_id_2, InternalFrame::world_frame_id(), Role::kIllustration),
            0);
  EXPECT_EQ(geometry_state_.GetNumGeometriesWithRole(Role::kIllustration),
            expected_count);
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            single_tree_total_geometry_count());

  // Case: Operate on the world frame with a source that *does* have anchored
  // geometry. Should remove the role from the single geometry.
  EXPECT_EQ(geometry_state_.RemoveRole(
      source_id_, InternalFrame::world_frame_id(), Role::kIllustration),
            1);
  EXPECT_EQ(geometry_state_.GetNumGeometriesWithRole(Role::kIllustration),
            --expected_count);
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            single_tree_total_geometry_count());
}

// Tests that GeometryState's bookkeeping is consistent after removal of the
// perception role from a _single_ geometry. Similar to the removal of
// the illustration role, but must also confirm coordination of render index
// values. This doesn't test the frame version because the frame-to-geometry
// logic was already tested in RemoveIllustrationFromFrame.
TEST_F(GeometryStateTest, RemovePerceptionFromGeometry) {
  SetUpSingleSourceTree();

  int expected_count = single_tree_total_geometry_count();
  // Relies on all geometries having perception properties.
  EXPECT_EQ(geometry_state_.GetNumGeometriesWithRole(Role::kPerception),
            expected_count);

  // Case: Remove perception from first dynamic geometry causes *anchored*
  // geometry render index to be moved.
  {
    const GeometryId id = geometries_[0];
    const InternalGeometry* geometry = gs_tester_.GetGeometry(id);
    EXPECT_TRUE(geometry->has_perception_role());
    RenderIndex target_index = *geometry->render_index(dummy_render_name_);
    // Set the dummy render engine to report that the anchored geometry has
    // moved.
    render_engine_->set_moved_render_index(
        gs_tester_.GetGeometry(anchored_geometry_)
            ->render_index(dummy_render_name_));
    EXPECT_EQ(geometry_state_.RemoveRole(source_id_, id, Role::kPerception), 1);
    EXPECT_EQ(geometry_state_.GetNumGeometriesWithRole(Role::kPerception),
              --expected_count);
    EXPECT_FALSE(geometry->has_perception_role());
    EXPECT_EQ(*gs_tester_.GetGeometry(anchored_geometry_)
                   ->render_index(dummy_render_name_),
              target_index);
  }

  // Case: Remove perception causes *dynamic* geometry to be moved.
  {
    const GeometryId id = geometries_[1];
    const InternalGeometry* geometry = gs_tester_.GetGeometry(id);
    EXPECT_TRUE(geometry->has_perception_role());
    RenderIndex target_index = *geometry->render_index(dummy_render_name_);
    // The second to last geometry added was the last dynamic geometry (i.e.,
    // the last id stored in geometries_. It should *now* have the render index
    // of the removed geometry.
    const InternalGeometry* last_geometry =
        gs_tester_.GetGeometry(geometries_.back());
    // These will ultimately be tested for equality; confirm they don't start
    // equal.
    EXPECT_NE(*last_geometry->render_index(dummy_render_name_), target_index);
    // Set the dummy render engine to report that the last dynamic geometry has
    // moved.
    render_engine_->set_moved_render_index(
        last_geometry->render_index(dummy_render_name_));
    EXPECT_EQ(geometry_state_.RemoveRole(source_id_, id, Role::kPerception), 1);
    EXPECT_EQ(geometry_state_.GetNumGeometriesWithRole(Role::kPerception),
              --expected_count);
    EXPECT_FALSE(geometry->has_perception_role());
    EXPECT_EQ(last_geometry->render_index(dummy_render_name_), target_index);
  }

  // Case: Removing role from a geometry that does not have that role has no
  // effect.
  {
    const GeometryId id = geometries_[0];
    EXPECT_EQ(geometry_state_.RemoveRole(source_id_, id, Role::kPerception), 0);
    EXPECT_EQ(geometry_state_.GetNumGeometriesWithRole(Role::kPerception),
              expected_count);
    EXPECT_FALSE(gs_tester_.GetGeometry(id)->has_perception_role());
  }
}

// Tests that exceptions are thrown under the documented circumstances for
// removing roles.
TEST_F(GeometryStateTest, RemoveRoleExceptions) {
  SetUpSingleSourceTree();

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
  SourceId source_id_2 = geometry_state_.RegisterNewSource("second_source");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveRole(source_id_2, frames_[0],
                                 Role::kUnassigned),
      std::logic_error, "Referenced .* frame doesn't belong to the source.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RemoveRole(source_id_2, geometries_[0],
                                 Role::kUnassigned),
      std::logic_error, ".*the geometry doesn't belong to that source.");
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
    std::vector<std::pair<Role, int>> roles{
        {Role::kProximity, num_proximity},
        {Role::kPerception, num_perception},
        {Role::kIllustration, num_illustration}};
    for (const auto& pair : roles) {
      const Role role = pair.first;
      const int expected_count = pair.second;
      const int actual_count = geometry_state_.NumGeometryWithRole(f_id, role);
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
  int perception_count = kGeometryCount;
  int illustration_count = kGeometryCount;
  FrameId f_id = frames_[0];
  ASSERT_TRUE(expected_roles(f_id, proximity_count, perception_count,
                             illustration_count));
}

}  // namespace
}  // namespace dev
}  // namespace geometry
}  // namespace drake
