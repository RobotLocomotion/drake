#include "drake/geometry/collision_group.h"

#include <set>
#include <unordered_set>
#include <vector>

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace {

// Helper class -- used to determine that the indicated identifiers are found
// in the CollisionGroup.

template <typename IdContainer>
void ExpectMembership(const CollisionGroup& group, const IdContainer& ids,
                      const char* failure_message) {
  for (auto id : ids) {
    EXPECT_TRUE(group.contains(id)) << failure_message;
  }
}

GTEST_TEST(CollisionGroupTests, DefaultConstructor) {
  CollisionGroup group;

  // Should report containing no frames or geometries.
  EXPECT_EQ(group.num_frames(), 0);
  EXPECT_EQ(group.num_geometries(), 0);

  // A random frame/geometry id will not be reported as being *in* the group.
  EXPECT_FALSE(group.contains(GeometryId::get_new_id()));
  EXPECT_FALSE(group.contains(FrameId::get_new_id()));
}

GTEST_TEST(CollisionGroupTests, ConversionConstructor) {
  GeometryId g_id = GeometryId::get_new_id();
  FrameId f_id = FrameId::get_new_id();
  auto geometry_list = {GeometryId::get_new_id(), GeometryId::get_new_id()};
  std::vector<GeometryId> geometry_vector = geometry_list;
  std::set<GeometryId> geometry_set{GeometryId::get_new_id(),
                                    GeometryId::get_new_id(),
                                    GeometryId::get_new_id()};
  std::unordered_set<GeometryId> geometry_hash{
      GeometryId::get_new_id(), GeometryId::get_new_id(),
      GeometryId::get_new_id(), GeometryId::get_new_id()};
  std::vector<FrameId> frame_vector{FrameId::get_new_id(),
                                    FrameId::get_new_id()};

  CollisionGroup group1(g_id);
  EXPECT_EQ(group1.num_frames(), 0);
  EXPECT_EQ(group1.num_geometries(), 1);
  EXPECT_TRUE(group1.contains(g_id));

  CollisionGroup group2(f_id);
  EXPECT_EQ(group2.num_frames(), 1);
  EXPECT_EQ(group2.num_geometries(), 0);
  EXPECT_TRUE(group2.contains(f_id));

  CollisionGroup group3(geometry_vector);
  EXPECT_EQ(group3.num_frames(), 0);
  EXPECT_EQ(group3.num_geometries(), static_cast<int>(geometry_vector.size()));
  ExpectMembership(group3, geometry_vector, "Vector constructor");

  CollisionGroup group4(geometry_set);
  EXPECT_EQ(group4.num_frames(), 0);
  EXPECT_EQ(group4.num_geometries(), static_cast<int>(geometry_set.size()));
  ExpectMembership(group4, geometry_set, "Set constructor");

  CollisionGroup group5(geometry_hash);
  EXPECT_EQ(group5.num_frames(), 0);
  EXPECT_EQ(group5.num_geometries(), static_cast<int>(geometry_hash.size()));
  ExpectMembership(group5, geometry_hash, "Unordered set constructor");

  CollisionGroup group6({geometry_vector[0], geometry_vector[1]});
  EXPECT_EQ(group6.num_frames(), 0);
  EXPECT_EQ(group6.num_geometries(), static_cast<int>(geometry_vector.size()));
  ExpectMembership(group6, geometry_vector,
                   "Geometry initializer list constructor");

  CollisionGroup group7({frame_vector[0], frame_vector[1]});
  EXPECT_EQ(group7.num_frames(), static_cast<int>(frame_vector.size()));
  EXPECT_EQ(group7.num_geometries(), 0);
  ExpectMembership(group7, frame_vector,
                   "Frame initializer list constructor");

  CollisionGroup group8(geometry_list);
  EXPECT_EQ(group8.num_frames(), 0);
  EXPECT_EQ(group8.num_geometries(), static_cast<int>(geometry_vector.size()));
  ExpectMembership(group8, geometry_vector,
                   "Initializer list lvalue constructor");

  CollisionGroup group9(geometry_list, frame_vector);
  EXPECT_EQ(group9.num_frames(), static_cast<int>(frame_vector.size()));
  EXPECT_EQ(group9.num_geometries(), static_cast<int>(geometry_vector.size()));
  ExpectMembership(group9, geometry_vector,
                   "Geometry initializer list, frame vector constructor");

  CollisionGroup group10(geometry_vector, {frame_vector[0], frame_vector[1]});
  EXPECT_EQ(group10.num_frames(), static_cast<int>(frame_vector.size()));
  EXPECT_EQ(group10.num_geometries(), static_cast<int>(geometry_vector.size()));
  ExpectMembership(group10, geometry_vector,
                   "Geometry vector, frame initializer list constructor");
}

GTEST_TEST(CollisionGroupTests, SingleFrameAdd) {
  CollisionGroup group;

  FrameId f = FrameId::get_new_id();
  EXPECT_FALSE(group.contains(f));
  group.Add(f);
  EXPECT_EQ(group.num_frames(), 1);
  EXPECT_TRUE(group.contains(f));

  // Adding a frame redundantly should not be a problem.
  EXPECT_NO_THROW(group.Add(f));
  // It should also not change the membership.
  EXPECT_EQ(group.num_frames(), 1);
  EXPECT_TRUE(group.contains(f));
}

GTEST_TEST(CollisionGroupTests, SingleGeometryAdd) {
  CollisionGroup group;

  GeometryId g = GeometryId::get_new_id();
  EXPECT_FALSE(group.contains(g));
  group.Add(g);
  EXPECT_EQ(group.num_geometries(), 1);
  EXPECT_TRUE(group.contains(g));

  // Adding a geometry redundantly should not be a problem.
  EXPECT_NO_THROW(group.Add(g));
  // It should also not change the membership.
  EXPECT_EQ(group.num_geometries(), 1);
  EXPECT_TRUE(group.contains(g));
}

GTEST_TEST(CollisionGroupTests, IterableFrameAdd) {
  CollisionGroup group;

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
  EXPECT_EQ(group.num_frames(), expected_num_frames);

  group.Add(frames_vector);
  expected_num_frames += static_cast<int>(frames_vector.size());
  EXPECT_EQ(group.num_frames(), expected_num_frames);
  ExpectMembership(group, frames_vector, "IterableFrameAdd - frames_vector");

  group.Add(frames_set);
  expected_num_frames += static_cast<int>(frames_set.size());
  EXPECT_EQ(group.num_frames(), expected_num_frames);
  ExpectMembership(group, frames_set, "IterableFrameAdd - frames_set");

  group.Add(frames_hash);
  expected_num_frames += static_cast<int>(frames_hash.size());
  EXPECT_EQ(group.num_frames(), expected_num_frames);
  ExpectMembership(group, frames_hash, "IterableFrameAdd - frames_hash");

  CollisionGroup group2;
  group2.Add({frames_vector[0], frames_vector[1]});
  EXPECT_EQ(group2.num_frames(), 2);
  ExpectMembership(group, frames_vector, "IterableFrameAdd - initializer_list");
}

GTEST_TEST(CollisionGroupTests, IterableGeometryAdd) {
  CollisionGroup group;

  // Each has a *different* number of geometries so that the result of each can
  // be easily distinguished by count.
  std::vector<GeometryId> geometry_vector{GeometryId::get_new_id(),
                                          GeometryId::get_new_id()};
  std::set<GeometryId> geometry_set{GeometryId::get_new_id(),
                                    GeometryId::get_new_id(),
                                    GeometryId::get_new_id()};
  std::unordered_set<GeometryId> geometry_hash{
      GeometryId::get_new_id(), GeometryId::get_new_id(),
      GeometryId::get_new_id(), GeometryId::get_new_id()};

  int expected_num_geometries = 0;
  EXPECT_EQ(group.num_frames(), expected_num_geometries);

  group.Add(geometry_vector);
  expected_num_geometries += static_cast<int>(geometry_vector.size());
  EXPECT_EQ(group.num_geometries(), expected_num_geometries);
  ExpectMembership(group, geometry_vector,
                   "IterableGeometryAdd - geometry_vector");

  group.Add(geometry_set);
  expected_num_geometries += static_cast<int>(geometry_set.size());
  EXPECT_EQ(group.num_geometries(), expected_num_geometries);
  ExpectMembership(group, geometry_set,
                   "IterableGeometryAdd - geometry_set");

  group.Add(geometry_hash);
  expected_num_geometries += static_cast<int>(geometry_hash.size());
  EXPECT_EQ(group.num_geometries(), expected_num_geometries);
  ExpectMembership(group, geometry_hash,
                   "IterableGeometryAdd - geometry_hash");

  CollisionGroup group2;
  group2.Add({geometry_vector[0], geometry_vector[1]});
  EXPECT_EQ(group2.num_geometries(), 2);
  ExpectMembership(group, geometry_vector,
                   "IterableGeometryAdd - initializer_list");
}

GTEST_TEST(CollisionGroupTests, IterableGeometryAndFrames) {
  CollisionGroup group;
  std::vector<GeometryId> geometry_vector{GeometryId::get_new_id(),
                                          GeometryId::get_new_id()};
  std::set<FrameId> frames_set{FrameId::get_new_id(), FrameId::get_new_id()};

  group.Add(geometry_vector, frames_set);
  EXPECT_EQ(group.num_frames(), static_cast<int>(frames_set.size()));
  EXPECT_EQ(group.num_geometries(), static_cast<int>(geometry_vector.size()));
  ExpectMembership(group, geometry_vector,
                   "IterableGeometryAndFrames - both containers - "
                       "geometry_vector");
  ExpectMembership(group, frames_set,
                   "IterableGeometryAndFrames - both containers - frames_set");

  CollisionGroup group4;
  group4.Add({geometry_vector[0], geometry_vector[1]}, frames_set);
  EXPECT_EQ(group4.num_frames(), static_cast<int>(frames_set.size()));
  EXPECT_EQ(group4.num_geometries(), static_cast<int>(geometry_vector.size()));
  ExpectMembership(
      group4, geometry_vector,
      "IterableGeometryAndFrames - geometry initializer list - geometry");
  ExpectMembership(group4, frames_set,
                   "IterableGeometryAndFrames - geometry initializer list - "
                       "frames_set");

  CollisionGroup group6;
  group6.Add(geometry_vector, {*frames_set.begin(), *(++frames_set.begin())});
  EXPECT_EQ(group6.num_frames(), static_cast<int>(frames_set.size()));
  EXPECT_EQ(group6.num_geometries(), static_cast<int>(geometry_vector.size()));
  ExpectMembership(group6, geometry_vector,
                   "IterableGeometryAndFrames - frame initializer list - "
                       "geometry_vector");
  ExpectMembership(group6, frames_set,
                   "IterableGeometryAndFrames - frame initializer list - "
                       "frame initializer list");
}

}  // namespace
}  // namespace geometry
}  // namespace drake
