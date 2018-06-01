#include "drake/geometry/geometry_set.h"

#include <set>
#include <unordered_set>
#include <vector>

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace {

// Helper class -- used to determine that the indicated identifiers are found
// in the GeometrySet.

template <typename IdContainer>
void ExpectContainsAll(const GeometrySet& geometry_set, const IdContainer& ids,
                       const char* failure_message) {
  for (auto id : ids) {
    EXPECT_TRUE(geometry_set.contains(id)) << failure_message;
  }
}

GTEST_TEST(GeometrySetTests, DefaultConstructor) {
  GeometrySet geometry_set;

  // Should report containing no frames or geometries.
  EXPECT_EQ(geometry_set.num_frames(), 0);
  EXPECT_EQ(geometry_set.num_geometries(), 0);

  // A random frame/geometry id will not be reported as being *in* the set.
  EXPECT_FALSE(geometry_set.contains(GeometryId::get_new_id()));
  EXPECT_FALSE(geometry_set.contains(FrameId::get_new_id()));
}

GTEST_TEST(GeometrySetTests, ConversionConstructor) {
  GeometryId g_id = GeometryId::get_new_id();
  FrameId f_id = FrameId::get_new_id();
  // NOTE: Subsequent tests depend on
  //   1) the vector of ids to be constructed from the initializer list of ids,
  //      and
  //   2) The initializer list has *two* ids.
  // This applies to both the vector of frame ids as well as the vector of
  // geometry ids. A change to either of those properties will require a change
  // of tests below (sets 6 & 7 for geometry ids and sets 8 & 9 for frame ids).
  auto geometry_list = {GeometryId::get_new_id(), GeometryId::get_new_id()};
  std::vector<GeometryId> geometry_vector = geometry_list;
  std::set<GeometryId> geometry_set{GeometryId::get_new_id(),
                                    GeometryId::get_new_id(),
                                    GeometryId::get_new_id()};
  std::unordered_set<GeometryId> geometry_hash{
      GeometryId::get_new_id(), GeometryId::get_new_id(),
      GeometryId::get_new_id(), GeometryId::get_new_id()};
  auto frame_list = {FrameId::get_new_id(), FrameId::get_new_id()};
  std::vector<FrameId> frame_vector{frame_list};

  auto test_single_source_constructor = [](
      const GeometrySet& set, const auto& source, int expected_frame_count,
      int expected_geometry_count, const char* message) {
    EXPECT_EQ(set.num_frames(), expected_frame_count) << message;
    EXPECT_EQ(set.num_geometries(), expected_geometry_count) << message;
    ExpectContainsAll(set, source, message);
  };

  GeometrySet set1(g_id);
  test_single_source_constructor(set1, std::set<GeometryId>{g_id}, 0, 1,
                                 "Single geometry id constructor");

  GeometrySet set2(f_id);
  test_single_source_constructor(set2, std::set<FrameId>{f_id}, 1, 0,
                                 "Single frame id constructor");

  GeometrySet set3(geometry_vector);
  test_single_source_constructor(set3, geometry_vector, 0,
                                 static_cast<int>(geometry_vector.size()),
                                 "Vector constructor");

  GeometrySet set4(geometry_set);
  test_single_source_constructor(set4, geometry_set, 0,
                                 static_cast<int>(geometry_set.size()),
                                 "Set constructor");

  GeometrySet set5(geometry_hash);
  test_single_source_constructor(set5, geometry_hash, 0,
                                 static_cast<int>(geometry_hash.size()),
                                 "Unordered set constructor");

  // These next two tests with initializer lists rely on the fact that the
  // geometry vector was initialized from `geometry_list`.
  GeometrySet set6(geometry_list);
  test_single_source_constructor(
      set6, geometry_list, 0, static_cast<int>(geometry_vector.size()),
      "GeometryId initializer list lvalue constructor");

  GeometrySet set7({geometry_vector[0], geometry_vector[1]});
  test_single_source_constructor(
      set7, geometry_vector, 0, static_cast<int>(geometry_vector.size()),
      "GeometryId initializer list rvalue constructor");

  GeometrySet set8(frame_list);
  test_single_source_constructor(set8, frame_vector,
                                 static_cast<int>(frame_vector.size()), 0,
                                 "FrameId initializer list lvalue constructor");

  GeometrySet set9({frame_vector[0], frame_vector[1]});
  test_single_source_constructor(set9, frame_vector,
                                 static_cast<int>(frame_vector.size()), 0,
                                 "FrameId initializer list rvalue constructor");

  // Constructors using geometry and frame id sets.
  GeometrySet set10(geometry_list, frame_vector);
  EXPECT_EQ(set10.num_frames(), static_cast<int>(frame_vector.size()));
  EXPECT_EQ(set10.num_geometries(), static_cast<int>(geometry_vector.size()));
  ExpectContainsAll(set10, geometry_vector,
                    "Geometry initializer list, frame vector constructor");
  ExpectContainsAll(set10, frame_vector,
                    "Geometry initializer list, frame vector constructor");

  GeometrySet set11(geometry_vector, {frame_vector[0], frame_vector[1]});
  EXPECT_EQ(set11.num_frames(), static_cast<int>(frame_vector.size()));
  EXPECT_EQ(set11.num_geometries(), static_cast<int>(geometry_vector.size()));
  ExpectContainsAll(set11, geometry_vector,
                    "Geometry vector, frame initializer list constructor");
  ExpectContainsAll(set11, frame_vector,
                    "Geometry vector, frame initializer list constructor");
}

GTEST_TEST(GeometrySetTests, SingleFrameAdd) {
  GeometrySet geometry_set;

  FrameId f = FrameId::get_new_id();
  EXPECT_FALSE(geometry_set.contains(f));
  geometry_set.Add(f);
  EXPECT_EQ(geometry_set.num_frames(), 1);
  EXPECT_TRUE(geometry_set.contains(f));

  // Adding a frame redundantly should not be a problem.
  EXPECT_NO_THROW(geometry_set.Add(f));
  // It should also not change the membership.
  EXPECT_EQ(geometry_set.num_frames(), 1);
  EXPECT_TRUE(geometry_set.contains(f));
}

GTEST_TEST(GeometrySetTests, SingleGeometryAdd) {
  GeometrySet geometry_set;

  GeometryId g = GeometryId::get_new_id();
  EXPECT_FALSE(geometry_set.contains(g));
  geometry_set.Add(g);
  EXPECT_EQ(geometry_set.num_geometries(), 1);
  EXPECT_TRUE(geometry_set.contains(g));

  // Adding a geometry redundantly should not be a problem.
  EXPECT_NO_THROW(geometry_set.Add(g));
  // It should also not change the membership.
  EXPECT_EQ(geometry_set.num_geometries(), 1);
  EXPECT_TRUE(geometry_set.contains(g));
}

GTEST_TEST(GeometrySetTests, IterableFrameAdd) {
  GeometrySet geometry_set;

  // Each has a *different* number of frames so that the result of each can be
  // easily distinguished by count.
  std::vector<FrameId> frames_vector{FrameId::get_new_id(),
                                     FrameId::get_new_id()};
  std::set<FrameId> frames_set{FrameId::get_new_id(), FrameId::get_new_id(),
                               FrameId::get_new_id()};
  std::unordered_set<FrameId> frames_hash{
      FrameId::get_new_id(), FrameId::get_new_id(), FrameId::get_new_id(),
      FrameId::get_new_id()};

  int expected_num_frames = 0;
  EXPECT_EQ(geometry_set.num_frames(), expected_num_frames);

  geometry_set.Add(frames_vector);
  expected_num_frames += static_cast<int>(frames_vector.size());
  EXPECT_EQ(geometry_set.num_frames(), expected_num_frames);
  ExpectContainsAll(geometry_set, frames_vector,
                    "IterableFrameAdd - frames_vector");

  geometry_set.Add(frames_set);
  expected_num_frames += static_cast<int>(frames_set.size());
  EXPECT_EQ(geometry_set.num_frames(), expected_num_frames);
  ExpectContainsAll(geometry_set, frames_set, "IterableFrameAdd - frames_set");

  geometry_set.Add(frames_hash);
  expected_num_frames += static_cast<int>(frames_hash.size());
  EXPECT_EQ(geometry_set.num_frames(), expected_num_frames);
  ExpectContainsAll(geometry_set, frames_hash,
                    "IterableFrameAdd - frames_hash");

  GeometrySet geometry_set2;
  geometry_set2.Add({frames_vector[0], frames_vector[1]});
  EXPECT_EQ(geometry_set2.num_frames(), 2);
  ExpectContainsAll(geometry_set, frames_vector,
                    "IterableFrameAdd - initializer_list");
}

GTEST_TEST(GeometrySetTests, IterableGeometryAdd) {
  GeometrySet geometry_set;

  // Each has a *different* number of geometries so that the result of each can
  // be easily distinguished by count.
  std::vector<GeometryId> geometry_id_vector{GeometryId::get_new_id(),
                                             GeometryId::get_new_id()};
  std::set<GeometryId> geometry_id_set{GeometryId::get_new_id(),
                                       GeometryId::get_new_id(),
                                       GeometryId::get_new_id()};
  std::unordered_set<GeometryId> geometry_id_hash{
      GeometryId::get_new_id(), GeometryId::get_new_id(),
      GeometryId::get_new_id(), GeometryId::get_new_id()};

  int expected_num_geometries = 0;
  EXPECT_EQ(geometry_set.num_frames(), expected_num_geometries);

  geometry_set.Add(geometry_id_vector);
  expected_num_geometries += static_cast<int>(geometry_id_vector.size());
  EXPECT_EQ(geometry_set.num_geometries(), expected_num_geometries);
  ExpectContainsAll(geometry_set, geometry_id_vector,
                    "IterableGeometryAdd - geometry_id_vector");

  geometry_set.Add(geometry_id_set);
  expected_num_geometries += static_cast<int>(geometry_id_set.size());
  EXPECT_EQ(geometry_set.num_geometries(), expected_num_geometries);
  ExpectContainsAll(geometry_set, geometry_id_set,
                    "IterableGeometryAdd - geometry_id_set");

  geometry_set.Add(geometry_id_hash);
  expected_num_geometries += static_cast<int>(geometry_id_hash.size());
  EXPECT_EQ(geometry_set.num_geometries(), expected_num_geometries);
  ExpectContainsAll(geometry_set, geometry_id_hash,
                    "IterableGeometryAdd - geometry_id_hash");

  GeometrySet geometry_set2;
  geometry_set2.Add({geometry_id_vector[0], geometry_id_vector[1]});
  EXPECT_EQ(geometry_set2.num_geometries(), 2);
  ExpectContainsAll(geometry_set, geometry_id_vector,
                    "IterableGeometryAdd - initializer_list");
}

GTEST_TEST(GeometrySetTests, IterableGeometryAndFrames) {
  GeometrySet geometry_set;
  std::vector<GeometryId> geometry_id_vector{GeometryId::get_new_id(),
                                             GeometryId::get_new_id()};
  std::set<FrameId> frame_id_set{FrameId::get_new_id(), FrameId::get_new_id()};

  geometry_set.Add(geometry_id_vector, frame_id_set);
  EXPECT_EQ(geometry_set.num_frames(), static_cast<int>(frame_id_set.size()));
  EXPECT_EQ(geometry_set.num_geometries(),
            static_cast<int>(geometry_id_vector.size()));
  ExpectContainsAll(geometry_set, geometry_id_vector,
                    "IterableGeometryAndFrames - both containers - "
                    "geometry_id_vector");
  ExpectContainsAll(
      geometry_set, frame_id_set,
      "IterableGeometryAndFrames - both containers - frame_id_set");

  GeometrySet geometry_set2;
  geometry_set2.Add({geometry_id_vector[0], geometry_id_vector[1]},
                    frame_id_set);
  EXPECT_EQ(geometry_set2.num_frames(), static_cast<int>(frame_id_set.size()));
  EXPECT_EQ(geometry_set2.num_geometries(),
            static_cast<int>(geometry_id_vector.size()));
  ExpectContainsAll(
      geometry_set2, geometry_id_vector,
      "IterableGeometryAndFrames - geometry initializer list - geometry");
  ExpectContainsAll(geometry_set2, frame_id_set,
                    "IterableGeometryAndFrames - geometry initializer list - "
                    "frame_id_set");

  GeometrySet geometry_set3;
  geometry_set3.Add(geometry_id_vector,
                    {*frame_id_set.begin(), *(++frame_id_set.begin())});
  EXPECT_EQ(geometry_set3.num_frames(), static_cast<int>(frame_id_set.size()));
  EXPECT_EQ(geometry_set3.num_geometries(),
            static_cast<int>(geometry_id_vector.size()));
  ExpectContainsAll(geometry_set3, geometry_id_vector,
                    "IterableGeometryAndFrames - frame initializer list - "
                    "geometry_id_vector");
  ExpectContainsAll(geometry_set3, frame_id_set,
                    "IterableGeometryAndFrames - frame initializer list - "
                    "frame initializer list");
}

}  // namespace
}  // namespace geometry
}  // namespace drake
