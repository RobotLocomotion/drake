#include "drake/geometry/geometry_state.h"

#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/internal_frame.h"
#include "drake/geometry/test/expect_error_message.h"

namespace drake {
namespace geometry {

using std::vector;

// Implementation of friend class that allows me to peek into the geometry state
// to confirm invariants on the state's internal workings as a result of
// operations.

template <class T>
class GeometryStateTester {
  using State = GeometryState<T>;

 public:
  void set_state(const State* state) { state_ = state; }

  FrameId get_world_frame() {
    return internal::InternalFrame::get_world_frame_id();
  }

  const std::unordered_map<SourceId, FrameIdSet>& get_source_frame_id_map() {
    return state_->source_frame_id_map_;
  }

  const std::unordered_map<SourceId, FrameIdSet>& get_source_root_frame_map() {
    return state_->source_root_frame_map_;
  }

  const std::unordered_map<FrameId, internal::InternalFrame>& get_frames() {
    return state_->frames_;
  }

  const std::unordered_map<GeometryId, internal::InternalGeometry>&
  get_geometries() {
    return state_->geometries_;
  }

  const std::unordered_map<GeometryId, internal::InternalAnchoredGeometry>&
  get_anchored_geometries() {
    return state_->anchored_geometries_;
  }

 private:
  const State* state_;
};

namespace {

using std::make_unique;
using std::move;
using std::unique_ptr;

class GeometryStateTest : public ::testing::Test {
 protected:
  void SetUp() {
    frame_ = make_unique<GeometryFrame>();
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
  //                                        s_id
  //                                        ╱  ╲
  //                                       f0   f1
  //                                      ╱ │    ├───┬───┐
  //                                    g0  g1   │   │   │
  //                                             f2  g2  g3
  //                                            ╱ ╲
  //                                           g4 g5
  //
  // Frame configuration
  //  f0 is @ <1, 2, 3>, with a 90-degree rotation around x.
  //  f1 is @ <10, 20, 30>, with a 90-degree rotation around y.
  //  f2 is @ <-10, -20, -30>, with a -90-degree rotation around y.
  // Geometry configuration
  //  gi is at position <i + 1, 0, 0>, with a rotation of iπ/2 radians around
  //    the x-axis.
  // f2's pose is the inverse of f1. As such, for g4 & g5, the pose
  // relative to the parent frame f2 is the same as to the world, e.g.,
  // X_PG = X_WG.
  SourceId SetUpSingleSourceTree() {
    using std::to_string;

    source_id_ = NewSource();

    // Create f0.
    frames_.push_back(geometry_state_.RegisterFrame(
        source_id_, GeometryFrame()));

    // Create f1.
    frames_.push_back(geometry_state_.RegisterFrame(
        source_id_, GeometryFrame()));

    // Create f2.
    frames_.push_back(geometry_state_.RegisterFrame(
        source_id_, frames_[1], GeometryFrame()));

    // Add geometries to each frame.
    Isometry3<double> pose = Isometry3<double>::Identity();
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
        ++g_count;
      }
    }
    return source_id_;
  }

  // Reports characteristics of the dummy tree.
  int single_tree_frame_count() const { return kFrameCount; }
  int single_tree_geometry_count() const {
    return kFrameCount * kGeometryCount;
  }

  // This method confirms that the stored dummy identifiers don't map to any
  // registered source identifier. This should only be invoked for scenarios
  // where there is *only* the single source.
  void AssertSingleTreeCleared() {
    // Confirms frames have been cleared.
    for (int f = 0; f < kFrameCount; ++f) {
      EXPECT_ERROR_MESSAGE(geometry_state_.BelongsToSource(frames_[f],
                                                           source_id_),
                           std::logic_error,
                           "Referenced frame \\d+ has not been registered.");
    }
    // Confirms geometries have been cleared.
    for (int g = 0; g < kFrameCount * kGeometryCount; ++g) {
      EXPECT_ERROR_MESSAGE(geometry_state_.BelongsToSource(geometries_[g],
                                                           source_id_),
                           std::logic_error,
                           "Referenced geometry \\d+ has not been registered.");
    }
    EXPECT_EQ(gs_tester_.get_source_frame_id_map().at(source_id_).size(), 0);
    EXPECT_EQ(gs_tester_.get_source_frame_id_map().size(), 1);
    EXPECT_EQ(gs_tester_.get_source_root_frame_map().at(source_id_).size(), 0);
    EXPECT_EQ(gs_tester_.get_source_root_frame_map().size(), 1);
    EXPECT_EQ(gs_tester_.get_frames().size(), 0);
    EXPECT_EQ(gs_tester_.get_geometries().size(), 0);
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
  // The id of the single-source tree.
  SourceId source_id_;

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
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RegisterNewSource(name),
      std::logic_error,
      "Registering new source with duplicate name: Unique.");

  // Case: query with invalid source id.
  EXPECT_ERROR_MESSAGE(geometry_state_.get_source_name(SourceId::get_new_id()),
                       std::logic_error,
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
  SourceId false_id = SourceId::get_new_id();
  EXPECT_FALSE(geometry_state_.source_is_registered(false_id));
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
      EXPECT_EQ(frame.get_parent_frame_id(), parent_id);
      EXPECT_EQ(frame.get_child_frames().size(), num_child_frames);
      const auto& child_geometries = frame.get_child_geometries();
      EXPECT_EQ(child_geometries.size(), 2);
      EXPECT_NE(child_geometries.find(geometries_[i * 2]),
                                      child_geometries.end());
      EXPECT_NE(child_geometries.find(geometries_[i * 2 + 1]),
                                      child_geometries.end());
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
    }
  }
}

// Tests that an attempt to add a frame to an invalid source throws an exception
// with meaningful message.
TEST_F(GeometryStateTest, AddFrameToInvalidSource) {
  SourceId s_id = SourceId::get_new_id();  // This is not a registered source.
  ASSERT_ERROR_MESSAGE(geometry_state_.RegisterFrame(s_id, *frame_.get()),
                       std::logic_error,
                       "Referenced geometry source \\d+ is not registered.");
}

// Tests that a frame added to a valid source appears in the source's frames.
TEST_F(GeometryStateTest, AddFirstFrameToValidSource) {
  SourceId s_id = NewSource();
  FrameId fid = geometry_state_.RegisterFrame(s_id, *frame_.get());
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

// Tests the valid removal of an existing frame (and its attached geometry).
TEST_F(GeometryStateTest, RemoveFrame) {
  SourceId s_id = SetUpSingleSourceTree();
  EXPECT_EQ(geometry_state_.get_num_frames(), kFrameCount);
  EXPECT_EQ(geometry_state_.get_num_geometries(), kFrameCount * kGeometryCount);
  geometry_state_.RemoveFrame(s_id, frames_[0]);
  EXPECT_EQ(geometry_state_.get_num_frames(), kFrameCount - 1);
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            (kFrameCount -1)* kGeometryCount);

  ExpectSourceDoesNotHaveFrame(s_id, frames_[0]);
}

// Tests the removal of a frame that has other frames hanging on it.
TEST_F(GeometryStateTest, RemoveFrameTree) {
  SourceId s_id = SetUpSingleSourceTree();
  FrameId fid = geometry_state_.RegisterFrame(s_id, frames_[0], *frame_);
  EXPECT_EQ(geometry_state_.get_num_frames(), kFrameCount + 1);
  geometry_state_.RemoveFrame(s_id, frames_[0]);
  EXPECT_EQ(geometry_state_.get_num_frames(), kFrameCount - 1);

  ExpectSourceDoesNotHaveFrame(s_id, frames_[0]);
  ExpectSourceDoesNotHaveFrame(s_id, fid);
}

// Tests the removal of a frame whose parent is *not* the world frame.
TEST_F(GeometryStateTest, RemoveFrameLeaf) {
  SourceId s_id = SetUpSingleSourceTree();
  FrameId fid = geometry_state_.RegisterFrame(s_id, frames_[0], *frame_.get());
  EXPECT_EQ(geometry_state_.get_num_frames(), kFrameCount + 1);
  EXPECT_TRUE(gs_tester_.get_frames().at(frames_[0]).has_child(fid));
  geometry_state_.RemoveFrame(s_id, fid);
  EXPECT_EQ(geometry_state_.get_num_frames(), kFrameCount);
  ExpectSourceDoesNotHaveFrame(s_id, fid);
  EXPECT_FALSE(gs_tester_.get_frames().at(frames_[0]).has_child(fid));
}

// Tests the response to invalid invocations of RemoveFrame.
TEST_F(GeometryStateTest, RemoveFrameInvalid) {
  SourceId s_id = SetUpSingleSourceTree();

  // Case: Valid source, invalid frame.
  EXPECT_ERROR_MESSAGE(geometry_state_.RemoveFrame(s_id, FrameId::get_new_id()),
                       std::logic_error,
                       "Referenced frame \\d+ has not been registered.");

  // Case: Invalid source, valid frame.
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RemoveFrame(SourceId::get_new_id(), frames_[0]),
      std::logic_error,
      "Referenced geometry source \\d+ is not registered.");

  // Case: Valid source and frame, but frame does _not_ belong to source.
  SourceId s_id2 = geometry_state_.RegisterNewSource("new_source");
  FrameId frame_id = geometry_state_.RegisterFrame(s_id2, *frame_.get());
  EXPECT_EQ(geometry_state_.get_num_frames(), kFrameCount + 1);
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RemoveFrame(s_id, frame_id),
      std::logic_error,
      "Trying to remove frame \\d+ from source \\d+.+the frame doesn't "
      "belong.+");
}

// Tests registration of geometry on valid source and frame. This relies on the
// correctness of GeometryState::BelongsToSource(GeometryId, SourceId) and
// GeometryState::GetFrameId(GeometryId) and, therefore, implicitly tests them.
TEST_F(GeometryStateTest, RegisterGeometryGoodSource) {
  SourceId s_id = NewSource();
  FrameId f_id = geometry_state_.RegisterFrame(s_id, *frame_);
  GeometryId g_id = geometry_state_.RegisterGeometry(s_id, f_id,
                                                     move(instance_));
  EXPECT_EQ(geometry_state_.GetFrameId(g_id), f_id);
  EXPECT_TRUE(geometry_state_.BelongsToSource(g_id, s_id));

  EXPECT_TRUE(gs_tester_.get_frames().at(f_id).has_child(g_id));
  const auto& geometry = gs_tester_.get_geometries().at(g_id);
  EXPECT_TRUE(geometry.is_child_of_frame(f_id));
  EXPECT_FALSE(geometry.get_parent_id());
}

// Tests registration of geometry on invalid source.
TEST_F(GeometryStateTest, RegisterGeometryMissingSource) {
  SourceId s_id = SourceId::get_new_id();
  FrameId f_id = FrameId::get_new_id();
  EXPECT_ERROR_MESSAGE(geometry_state_.RegisterGeometry(s_id, f_id,
                                                        move(instance_)),
                       std::logic_error,
                       "Referenced geometry source \\d+ is not registered.");
}

// Tests registration of geometry on valid source and non-existant frame.
TEST_F(GeometryStateTest, RegisterGeometryMissingFrame) {
  SourceId s_id = NewSource();

  FrameId f_id = FrameId::get_new_id();
  EXPECT_ERROR_MESSAGE(geometry_state_.RegisterGeometry(s_id, f_id,
                                                        move(instance_)),
                       std::logic_error,
                       "Referenced frame \\d+ for source \\d+\\,"
                           " but the frame doesn't belong to the source.");
}

// Tests error resulting from passing a null GeometryInstance.
TEST_F(GeometryStateTest, RegisterNullGeometry) {
  SourceId s_id = NewSource();
  FrameId f_id = geometry_state_.RegisterFrame(s_id, *frame_);
  unique_ptr<GeometryInstance> null_geometry;
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RegisterGeometry(s_id, f_id, move(null_geometry)),
      std::logic_error,
      "Registering null geometry to frame \\d+, on source \\d+.");
}

// Tests the logic for hanging a geometry on another geometry.
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
  GeometryId g_id =
      geometry_state_.RegisterGeometryWithParent(s_id,
                                                 parent_id,
                                                 move(instance));

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
// on a non-existant geometry id.
TEST_F(GeometryStateTest, RegisterGeometryonInvalidGeometry) {
  SourceId s_id = SetUpSingleSourceTree();
  Isometry3<double> pose = Isometry3<double>::Identity();
  auto instance =
      make_unique<GeometryInstance>(pose, unique_ptr<Shape>(new Sphere(1)));
  GeometryId junk_id = GeometryId::get_new_id();
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RegisterGeometryWithParent(s_id, junk_id, move(instance)),
      std::logic_error,
      "Referenced geometry \\d+ has not been registered.");
}

// Tests the response to passing a null pointer as a GeometryInstance.
TEST_F(GeometryStateTest, RegisterNullGeometryonGeometry) {
  SourceId s_id = SetUpSingleSourceTree();
  unique_ptr<GeometryInstance> instance;
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RegisterGeometryWithParent(s_id, geometries_[0],
                                                 move(instance)),
      std::logic_error,
      "Registering null geometry to geometry \\d+, on source \\d+.");
}

// Tests the RemoveGeometry functionality.
TEST_F(GeometryStateTest, RemoveGeometry) {
  SourceId s_id = SetUpSingleSourceTree();
  // The geometry to remove, and the frame to which it belongs.
  GeometryId g_id = geometries_[0];
  FrameId f_id = frames_[0];
  // Confirm that the first geometry belongs to the first frame.
  ASSERT_EQ(geometry_state_.GetFrameId(g_id), f_id);
  EXPECT_EQ(geometry_state_.get_num_geometries(), kFrameCount * kGeometryCount);
  geometry_state_.RemoveGeometry(s_id, g_id);
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            kFrameCount * kGeometryCount - 1);

  EXPECT_FALSE(gs_tester_.get_frames().at(f_id).has_child(g_id));
  EXPECT_EQ(gs_tester_.get_geometries().find(g_id),
            gs_tester_.get_geometries().end());
}

// Tests the RemoveGeometry functionality in which the geometry removed has
// geometry children.
TEST_F(GeometryStateTest, RemoveGeometryRecursiveParent) {
  SourceId s_id = SetUpSingleSourceTree();
  // The geometry to remove, and the frame to which it belongs.
  GeometryId root_id = geometries_[0];
  FrameId f_id = frames_[0];
  // Confirm that the first geometry belongs to the first frame.
  ASSERT_EQ(geometry_state_.GetFrameId(root_id), f_id);
  // Hang geometry from the first geometry.
  GeometryId g_id = geometry_state_.RegisterGeometryWithParent(
      s_id, root_id,
      make_unique<GeometryInstance>(Isometry3<double>::Identity(),
                                    unique_ptr<Shape>(new Sphere(1))));
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            kFrameCount * kGeometryCount + 1);
  EXPECT_EQ(geometry_state_.GetFrameId(g_id), f_id);

  geometry_state_.RemoveGeometry(s_id, root_id);
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            kFrameCount * kGeometryCount - 1);

  const auto& frame = gs_tester_.get_frames().at(f_id);
  EXPECT_FALSE(frame.has_child(root_id));
  EXPECT_FALSE(frame.has_child(g_id));
  EXPECT_EQ(gs_tester_.get_geometries().find(root_id),
            gs_tester_.get_geometries().end());
  EXPECT_EQ(gs_tester_.get_geometries().find(g_id),
            gs_tester_.get_geometries().end());
}

// Tests the RemoveGeometry functionality in which the geometry is a child of
// another geometry.
TEST_F(GeometryStateTest, RemoveGeometryRecursiveChild) {
  SourceId s_id = SetUpSingleSourceTree();
  // The geometry parent and frame to which it belongs.
  GeometryId parent_id = geometries_[0];
  FrameId frame_id = frames_[0];
  // Confirm that the first geometry belongs to the first frame.
  ASSERT_EQ(geometry_state_.GetFrameId(parent_id), frame_id);
  // Hang geometry from the first geometry.
  GeometryId g_id = geometry_state_.RegisterGeometryWithParent(
      s_id, parent_id,
      make_unique<GeometryInstance>(Isometry3<double>::Identity(),
                                    unique_ptr<Shape>(new Sphere(1))));
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            kFrameCount * kGeometryCount + 1);
  EXPECT_EQ(geometry_state_.GetFrameId(g_id), frame_id);

  geometry_state_.RemoveGeometry(s_id, g_id);
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            kFrameCount * kGeometryCount);
  EXPECT_EQ(geometry_state_.GetFrameId(parent_id), frame_id);

  EXPECT_FALSE(gs_tester_.get_frames().at(frame_id).has_child(g_id));
  EXPECT_TRUE(gs_tester_.get_frames().at(frame_id).has_child(parent_id));
  EXPECT_FALSE(gs_tester_.get_geometries().at(parent_id).has_child(g_id));
}

// Tests the response to invalid use of RemoveGeometry.
TEST_F(GeometryStateTest, RemoveGeometryInvalid) {
  SourceId s_id = SetUpSingleSourceTree();

  // Case: Invalid source id, valid geometry id.
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RemoveGeometry(SourceId::get_new_id(),
                                     geometries_[0]),
      std::logic_error,
      "Referenced geometry source \\d+ is not registered.");

  // Case: Invalid geometry id, valid source id.
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RemoveGeometry(s_id, GeometryId::get_new_id()),
      std::logic_error,
      "Referenced geometry \\d+ has not been registered.");

  // Case: Valid geometry and source, but geometry belongs to different source.
  SourceId s_id2 = geometry_state_.RegisterNewSource("new_source");
  FrameId frame_id = geometry_state_.RegisterFrame(s_id2, *frame_);
  EXPECT_EQ(geometry_state_.get_num_frames(), kFrameCount + 1);
  GeometryId g_id = geometry_state_.RegisterGeometry(
      s_id2, frame_id,
      make_unique<GeometryInstance>(Isometry3<double>::Identity(),
                                    unique_ptr<Shape>(new Sphere(1))));
  EXPECT_EQ(geometry_state_.get_num_geometries(),
            kFrameCount * kGeometryCount + 1);
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RemoveGeometry(s_id, g_id),
      std::logic_error,
      "Trying to remove geometry \\d+ from source \\d+.+geometry doesn't "
          "belong.+");
}

// Tests the registration of anchored geometry.
TEST_F(GeometryStateTest, RegisterAnchoredGeometry) {
  SourceId s_id = NewSource("new source");
  Isometry3<double> pose = Isometry3<double>::Identity();
  auto instance = make_unique<GeometryInstance>(
      pose, unique_ptr<Shape>(new Sphere(1)));
  auto g_id = geometry_state_.RegisterAnchoredGeometry(s_id, move(instance));
  EXPECT_TRUE(geometry_state_.BelongsToSource(g_id, s_id));
}

// Tests the attempt to register anchored geometry on an invalid source.
TEST_F(GeometryStateTest, RegisterAnchoredGeometryInvalidSource) {
  Isometry3<double> pose = Isometry3<double>::Identity();
  auto instance = make_unique<GeometryInstance>(
      pose, unique_ptr<Shape>(new Sphere(1)));
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RegisterAnchoredGeometry(SourceId::get_new_id(),
                                               move(instance)),
      std::logic_error,
      "Referenced geometry source \\d+ is not registered.");
}

// Tests the response of attempting to register a null pointer GeometryInstance
// as anchored geometry.
TEST_F(GeometryStateTest, RegisterAnchoredNullGeometry) {
  unique_ptr<GeometryInstance> instance;
  EXPECT_ERROR_MESSAGE(
      geometry_state_.RegisterAnchoredGeometry(SourceId::get_new_id(),
                                               move(instance)),
      std::logic_error,
      "Registering null anchored geometry on source \\d+.");
}

// Tests removal of anchored geometry.
TEST_F(GeometryStateTest, RemoveAnchoredGeometry) {
  SourceId s_id = SetUpSingleSourceTree();
  Vector3<double> normal{0, 1, 0};
  Vector3<double> point{1, 1, 1};
  auto anchored_id_1 = geometry_state_.RegisterAnchoredGeometry(
      s_id, make_unique<GeometryInstance>(HalfSpace::MakePose(normal, point),
                                          make_unique<HalfSpace>()));
  auto anchored_id_2 = geometry_state_.RegisterAnchoredGeometry(
      s_id, make_unique<GeometryInstance>(
                HalfSpace::MakePose(Vector3<double>{1, 0, 0},
                                    Vector3<double>{-1, 0, 0}),
                make_unique<HalfSpace>()));
  // Confirm conditions of having added two anchored geometries.
  EXPECT_TRUE(geometry_state_.BelongsToSource(anchored_id_1, s_id));
  EXPECT_TRUE(geometry_state_.BelongsToSource(anchored_id_2, s_id));
  EXPECT_EQ(geometry_state_.get_num_anchored_geometries(), 2);

  // Performs tested action.
  geometry_state_.RemoveGeometry(s_id, anchored_id_1);

  EXPECT_EQ(geometry_state_.get_num_anchored_geometries(), 1);
}

// Confirms the behavior for requesting geometry poses with a bad geometry
// identifier. The basic behavior is tested implicitly in other tests because
// they rely on them to validate state.
TEST_F(GeometryStateTest, GetPoseForBadGeometryId) {
  EXPECT_ERROR_MESSAGE(
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
  EXPECT_ERROR_MESSAGE(geometry_state_.BelongsToSource(FrameId::get_new_id(),
                                                       source_id),
                       std::logic_error,
                       "Referenced geometry source \\d+ is not registered.");
  EXPECT_ERROR_MESSAGE(geometry_state_.BelongsToSource(GeometryId::get_new_id(),
                                                       source_id),
                       std::logic_error,
                       "Referenced geometry source \\d+ is not registered.");
  SetUpSingleSourceTree();
  GeometryId anchored_id = geometry_state_.RegisterAnchoredGeometry(
      source_id_,
      make_unique<GeometryInstance>(Isometry3<double>::Identity(),
                                    std::unique_ptr<Shape>(new Sphere(1))));
  // Valid frame/geometry ids.
  EXPECT_ERROR_MESSAGE(geometry_state_.BelongsToSource(frames_[0],
                                                       source_id),
                       std::logic_error,
                       "Referenced geometry source \\d+ is not registered.");
  EXPECT_ERROR_MESSAGE(geometry_state_.BelongsToSource(geometries_[0],
                                                       source_id),
                       std::logic_error,
                       "Referenced geometry source \\d+ is not registered.");
  EXPECT_ERROR_MESSAGE(geometry_state_.BelongsToSource(anchored_id,
                                                       source_id),
                       std::logic_error,
                       "Referenced geometry source \\d+ is not registered.");
}

// This tests the source ownership functionality for frames - a function which
// reports if a frame belongs to the specified source.
TEST_F(GeometryStateTest, SourceOwnershipFrameId) {
  SourceId s_id = SetUpSingleSourceTree();
  // Test for invalid frame.
  EXPECT_ERROR_MESSAGE(geometry_state_.BelongsToSource(FrameId::get_new_id(),
                                                       s_id),
                       std::logic_error,
                       "Referenced frame \\d+ has not been registered.");
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
  EXPECT_ERROR_MESSAGE(geometry_state_.BelongsToSource(GeometryId::get_new_id(),
                                                       s_id),
                       std::logic_error,
                       "Referenced geometry \\d+ has not been registered.");
  // Test for valid geometry.
  EXPECT_TRUE(geometry_state_.BelongsToSource(geometries_[0], s_id));
  EXPECT_TRUE(geometry_state_.BelongsToSource(anchored_id, s_id));
}

// This confirms the failure state of calling GeometryState::GetFrameId with a
// bad geometry identifier.
TEST_F(GeometryStateTest, GetFrameIdFromBadId) {
  EXPECT_ERROR_MESSAGE(geometry_state_.GetFrameId(GeometryId::get_new_id()),
                       std::logic_error,
                       "Referenced geometry \\d+ has not been registered.");
}

// This tests that clearing a source eliminates all of its geometry and frames,
// leaving the source registered.
TEST_F(GeometryStateTest, ClearSourceData) {
  EXPECT_ERROR_MESSAGE(geometry_state_.ClearSource(SourceId::get_new_id()),
                       std::logic_error,
                       "Referenced geometry source \\d+ is not registered.");

  SourceId s_id = SetUpSingleSourceTree();
  geometry_state_.ClearSource(s_id);
  EXPECT_TRUE(geometry_state_.source_is_registered(s_id));
  AssertSingleTreeCleared();
}

}  // namespace
}  // namespace geometry
}  // namespace drake
