#include "drake/geometry/geometry_state.h"

#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_set.h"
#include "drake/geometry/internal_frame.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {

using std::vector;

// Implementation of friend class that allows me to peek into the geometry state
// to confirm invariants on the state's internal workings as a result of
// operations.

template <class T>
class GeometryStateTester {
 public:
  void set_state(GeometryState<T>* state) { state_ = state; }

  FrameId get_world_frame() const {
    return internal::InternalFrame::get_world_frame_id();
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

  const std::unordered_map<GeometryId, internal::InternalAnchoredGeometry>&
  get_anchored_geometries() const {
    return state_->anchored_geometries_;
  }

  const vector<GeometryId>& get_geometry_index_id_map() const {
    return state_->geometry_index_id_map_;
  }

  const vector<GeometryId>& get_anchored_geometry_index_id_map() const {
    return state_->anchored_geometry_index_id_map_;
  }

  const vector<FrameId>& get_pose_index_frame_id_map() const {
    return state_->pose_index_to_frame_map_;
  }

  const vector<Isometry3<double>>& get_geometry_frame_poses() const {
    return state_->X_FG_;
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

 private:
  GeometryState<T>* state_;
};

namespace {

using std::make_unique;
using std::move;
using std::unique_ptr;

class GeometryStateTest : public ::testing::Test {
 protected:
  void SetUp() {
    frame_ = make_unique<GeometryFrame>("ref_frame",
                                        Isometry3<double>::Identity());
    instance_pose_.translation() << 10, 20, 30;
    instance_ = make_unique<GeometryInstance>(
        instance_pose_, unique_ptr<Shape>(new Sphere(1.0)));
    gs_tester_.set_state(&geometry_state_);
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
  SourceId SetUpSingleSourceTree() {
    using std::to_string;

    source_id_ = NewSource();

    // Create f0.
    Isometry3<double> pose = Isometry3<double>::Identity();
    pose.translation() << 1, 2, 3;
    pose.linear() << 1, 0, 0, 0, 0, 1, 0, -1, 0;  // 90° around x-axis.
    frames_.push_back(geometry_state_.RegisterFrame(
        source_id_, GeometryFrame("f0", pose)));
    X_WF_.push_back(pose);
    X_PF_.push_back(pose);

    // Create f1.
    pose.translation() << 10, 20, 30;
    pose.linear() << 0, 0, -1, 0, 1, 0, 1, 0, 0;  // 90° around y-axis.
    frames_.push_back(geometry_state_.RegisterFrame(
        source_id_, GeometryFrame("f1", pose)));
    X_WF_.push_back(pose);
    X_PF_.push_back(pose);

    // Create f2.
    pose = pose.inverse();
    frames_.push_back(geometry_state_.RegisterFrame(
        source_id_, frames_[1], GeometryFrame("f2", pose)));
    X_WF_.push_back(X_WF_[1] * pose);
    X_PF_.push_back(pose);

    // Add geometries to each frame.
    const Vector3<double> x_axis(1, 0, 0);
    geometries_.resize(kFrameCount * kGeometryCount);
    int g_count = 0;
    for (auto frame_id : frames_) {
      for (int i = 0; i < kGeometryCount; ++i) {
        pose.translation() << g_count + 1, 0, 0;
        pose.linear() =
            AngleAxis<double>(g_count * M_PI / 2.0, x_axis).matrix();
        geometries_[g_count] = geometry_state_.RegisterGeometry(
            source_id_, frame_id,
            make_unique<GeometryInstance>(
                pose, std::unique_ptr<Shape>(new Sphere(1))));
        X_FG_.push_back(pose);
        ++g_count;
      }
    }

    // Create anchored geometry.
    pose = Isometry3<double>::Identity();
    pose.translation() << 0, 0, -1;
    anchored_geometry_ = geometry_state_.RegisterAnchoredGeometry(
        source_id_, make_unique<GeometryInstance>(
                        pose, std::unique_ptr<Box>(new Box(100, 100, 2))));
    return source_id_;
  }

  // Reports characteristics of the dummy tree.
  int single_tree_frame_count() const { return kFrameCount; }

  int single_tree_geometry_count() const {
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
    EXPECT_EQ(gs_tester_.get_geometry_frame_poses().size(), 0);
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

  // Values for setting up and testing the dummy tree.
  enum Counts {
    kFrameCount = 3,
    kGeometryCount = 2    // Geometries *per frame*.
  };
  // The frame ids created in the dummy tree instantiation.
  vector<FrameId> frames_;
  // The geometry ids created in the dummy tree instantiation.
  vector<GeometryId> geometries_;
  // The single, anchored geometry id.
  GeometryId anchored_geometry_;
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
};

// Confirms that a new GeometryState has no data.
TEST_F(GeometryStateTest, Constructor) {
  EXPECT_EQ(geometry_state_.get_num_sources(), 0);
  EXPECT_EQ(geometry_state_.get_num_frames(), 0);
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
  EXPECT_EQ(geometry_state_.get_num_sources(), 1);
  EXPECT_EQ(geometry_state_.get_num_frames(), single_tree_frame_count());
  EXPECT_EQ(geometry_state_.get_num_geometries(), single_tree_geometry_count());
  EXPECT_EQ(geometry_state_.get_num_anchored_geometries(),
            anchored_geometry_count());
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
  EXPECT_EQ(ad_tester.get_anchored_geometries(),
            d_tester.get_anchored_geometries());

  // 2. Test the vectors of ids
  EXPECT_EQ(ad_tester.get_geometry_index_id_map(),
            d_tester.get_geometry_index_id_map());
  EXPECT_EQ(ad_tester.get_geometry_index_id_map(),
            d_tester.get_geometry_index_id_map());
  EXPECT_EQ(ad_tester.get_anchored_geometry_index_id_map(),
            d_tester.get_anchored_geometry_index_id_map());
  EXPECT_EQ(ad_tester.get_pose_index_frame_id_map(),
            d_tester.get_pose_index_frame_id_map());

  // 3. Compare Isometry3<double> with Isometry3<double>
  EXPECT_EQ(ad_tester.get_geometry_frame_poses().size(),
            d_tester.get_geometry_frame_poses().size());
  for (size_t i = 0; i < ad_tester.get_geometry_frame_poses().size(); ++i) {
    EXPECT_TRUE(CompareMatrices(
        ad_tester.get_geometry_frame_poses()[i].matrix().block<3, 4>(0, 0),
        d_tester.get_geometry_frame_poses()[i].matrix().block<3, 4>(0, 0)));
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
  SourceId s_id = SetUpSingleSourceTree();

  // The source has *direct* access to all registered frames.
  {
    const auto& s_f_id_map = gs_tester_.get_source_frame_id_map();
    EXPECT_EQ(s_f_id_map.size(), 1);
    EXPECT_NE(s_f_id_map.find(s_id), s_f_id_map.end());
    const auto &f_id_set = s_f_id_map.at(s_id);
    for (int f = 0; f < kFrameCount; ++f) {
      EXPECT_NE(f_id_set.find(frames_[f]), f_id_set.end());
    }
  }

  // The root map *only* includes the root frames. Frames 0 & 1 *should* be
  // included; frame 2 should *not*.
  {
    const auto& s_root_map = gs_tester_.get_source_root_frame_map();
    EXPECT_EQ(s_root_map.size(), 1);
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
    EXPECT_EQ(internal_frames.size(), kFrameCount);
    auto test_frame = [internal_frames, this, s_id](int i, FrameId parent_id,
                                                    int num_child_frames) {
      const auto& frame = internal_frames.at(frames_[i]);
      EXPECT_EQ(frame.get_source_id(), s_id);
      EXPECT_EQ(frame.get_id(), frames_[i]);
      EXPECT_EQ(frame.get_name(), "f" + to_string(i));
      EXPECT_EQ(frame.get_frame_group(), 0);  // Defaults to zero.
      EXPECT_EQ(frame.get_pose_index(), i);   // ith frame added.
      EXPECT_EQ(frame.get_parent_frame_id(), parent_id);
      EXPECT_EQ(frame.get_child_frames().size(), num_child_frames);
      const auto& child_geometries = frame.get_child_geometries();
      EXPECT_EQ(child_geometries.size(), 2);
      EXPECT_NE(child_geometries.find(geometries_[i * 2]),
                                      child_geometries.end());
      EXPECT_NE(child_geometries.find(geometries_[i * 2 + 1]),
                                      child_geometries.end());
      const auto& frame_in_parent = gs_tester_.get_frame_parent_poses();
      EXPECT_TRUE(
          CompareMatrices(frame_in_parent[frame.get_pose_index()].matrix(),
                          X_PF_[i].matrix()));
    };
    test_frame(0, gs_tester_.get_world_frame(), 0);
    test_frame(1, gs_tester_.get_world_frame(), 1);
    test_frame(2, frames_[1], 0);
  }

  // The internal geometries are what and where they should be.
  {
    const auto& internal_geometries = gs_tester_.get_geometries();
    EXPECT_EQ(internal_geometries.size(), kFrameCount * kGeometryCount);
    for (int i = 0; i < kFrameCount * kGeometryCount; ++i) {
      const auto& geometry = internal_geometries.at(geometries_[i]);
      EXPECT_EQ(geometry.get_frame_id(), frames_[i / kGeometryCount]);
      EXPECT_EQ(geometry.get_id(), geometries_[i]);
      EXPECT_EQ(geometry.get_child_geometry_ids().size(), 0);
      EXPECT_FALSE(geometry.get_parent_id());
      // TODO(SeanCurtis-TRI): Update this when names are being used.
      EXPECT_EQ(geometry.get_engine_index(), i);
      EXPECT_EQ(geometry.get_child_geometry_ids().size(), 0);
      EXPECT_FALSE(geometry.get_parent_id());

      // Note: There are no geometries parented to other geometries. The results
      // of GetPoseInFrame() and GetPoseInParent() must be the identical (as
      // the documentation for GeometryState::GetPoseInParent() indicates).
      EXPECT_TRUE(CompareMatrices(
          geometry_state_.GetPoseInFrame(geometry.get_id()).matrix(),
          X_FG_[i].matrix()));
      EXPECT_TRUE(CompareMatrices(
          geometry_state_.GetPoseInParent(geometry.get_id()).matrix(),
          X_FG_[i].matrix()));

      EXPECT_EQ(
          gs_tester_.get_geometry_index_id_map()[geometry.get_engine_index()],
          geometry.get_id());
    }
  }
  EXPECT_EQ(gs_tester_.get_geometry_frame_poses().size(),
            kFrameCount * kGeometryCount);
  EXPECT_EQ(gs_tester_.get_geometry_world_poses().size(),
            kFrameCount * kGeometryCount);
  EXPECT_EQ(gs_tester_.get_frame_parent_poses().size(), kFrameCount);
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
  EXPECT_EQ(geometry_state_.get_num_frames(), kFrameCount + 1);
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
    // This should remove exactly one element.
    EXPECT_EQ(all_frames.erase(id), 1);
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
  CompareMatrices(X_FG.matrix(), instance_pose_.matrix());

  EXPECT_TRUE(gs_tester_.get_frames().at(f_id).has_child(g_id));
  const auto& geometry = gs_tester_.get_geometries().at(g_id);
  EXPECT_TRUE(geometry.is_child_of_frame(f_id));
  EXPECT_FALSE(geometry.get_parent_id());
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
  auto instance =
      make_unique<GeometryInstance>(pose, unique_ptr<Shape>(new Sphere(1)));
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
  EXPECT_EQ(geometry.get_frame_id(), frame_id);
  EXPECT_TRUE(geometry.is_child_of_geometry(parent_id));
  EXPECT_TRUE(gs_tester_.get_geometries().at(parent_id).has_child(g_id));
}

// Tests the response to the erroneous action of trying to hang a new geometry
// on a non-existent geometry id.
TEST_F(GeometryStateTest, RegisterGeometryonInvalidGeometry) {
  SourceId s_id = SetUpSingleSourceTree();
  Isometry3<double> pose = Isometry3<double>::Identity();
  auto instance =
      make_unique<GeometryInstance>(pose, unique_ptr<Shape>(new Sphere(1)));
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
      pose, unique_ptr<Shape>(new Sphere(1)));
  GeometryId expected_g_id = instance->id();
  auto g_id = geometry_state_.RegisterAnchoredGeometry(s_id, move(instance));
  EXPECT_EQ(g_id, expected_g_id);
  EXPECT_TRUE(geometry_state_.BelongsToSource(g_id, s_id));
}

// Confirms that registering two geometries with the same id causes failure.
TEST_F(GeometryStateTest, RegisterDuplicateAnchoredGeometry) {
  SourceId s_id = NewSource();
  auto instance_copy = make_unique<GeometryInstance>(*instance_);
  geometry_state_.RegisterAnchoredGeometry(s_id, move(instance_));
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.RegisterAnchoredGeometry(s_id, move(instance_copy)),
      std::logic_error,
      "Registering anchored geometry with an id that has already been "
      "registered: \\d+");
}

// Tests the attempt to register anchored geometry on an invalid source.
TEST_F(GeometryStateTest, RegisterAnchoredGeometryInvalidSource) {
  Isometry3<double> pose = Isometry3<double>::Identity();
  auto instance = make_unique<GeometryInstance>(
      pose, unique_ptr<Shape>(new Sphere(1)));
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
      "Registering null anchored geometry on source \\d+.");
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
                                    std::unique_ptr<Shape>(new Sphere(1))));
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
      s_id,
      make_unique<GeometryInstance>(
          Isometry3<double>::Identity(),
          std::unique_ptr<Shape>(new Sphere(1))));
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
  std::vector<FrameId> bad_frames;
  for (int i = 0; i < kFrameCount; ++i) {
    bad_frames.push_back(FrameId::get_new_id());
  }
  FramePoseVector<double> frame_set_2(s_id, bad_frames);
  DRAKE_EXPECT_THROWS_MESSAGE(
      gs_tester_.ValidateFrameIds(frame_set_2), std::runtime_error,
      "Registered frame id \\(\\d+\\) belonging to source \\d+ was not found "
          "in the provided kinematics data.");

  // Case: Too few frames.
  std::vector<FrameId> missing_frames;
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

  // Query frame group.
  EXPECT_EQ(geometry_state_.get_frame_group(frames_[0]), 0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.get_frame_group(FrameId::get_new_id()), std::logic_error,
      "No frame group available for invalid frame id: \\d+");

  // Query frame name.
  EXPECT_EQ(geometry_state_.get_frame_name(frames_[0]), "f0");
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
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.get_pose_in_parent(FrameId::get_new_id()),
      std::logic_error, "No pose available for invalid frame id: \\d+");
}

// Test disallowing collisions among members of a group (self collisions).
TEST_F(GeometryStateTest, ExcludeCollisionsWithin) {
  SetUpSingleSourceTree();

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
  SetUpSingleSourceTree();

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

// Tests the documented error conditions of ExcludeCollisionsWithin.
TEST_F(GeometryStateTest, SelfCollisionFilterExceptions) {
  SetUpSingleSourceTree();

  // NOTE: a collision group with a single frame or geometry doesn't exercise
  // self-collision filtering logic.
  GeometrySet set_bad_frame{FrameId::get_new_id(), FrameId::get_new_id()};
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.ExcludeCollisionsWithin(set_bad_frame), std::logic_error,
      "Geometry set includes a frame id that doesn't belong to the "
          "SceneGraph: \\d+");

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
      "Geometry set includes a frame id that doesn't belong to the "
          "SceneGraph: \\d+");
  DRAKE_EXPECT_THROWS_MESSAGE(
      geometry_state_.ExcludeCollisionsBetween(set_good_frame,
                                               set_bad_frame),
      std::logic_error,
      "Geometry set includes a frame id that doesn't belong to the "
          "SceneGraph: \\d+");

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

}  // namespace
}  // namespace geometry
}  // namespace drake
