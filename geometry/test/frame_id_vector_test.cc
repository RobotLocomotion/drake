#include "drake/geometry/frame_id_vector.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/test_utilities/expect_error_message.h"

namespace drake {
namespace geometry {
namespace {

using std::vector;

// Simply tests successful construction.
GTEST_TEST(FrameIdVector, ConstructorSuccess) {
  SourceId source_id = SourceId::get_new_id();

  // Case: Empty set.
  FrameIdVector ids1(source_id);
  EXPECT_EQ(ids1.get_source_id(), source_id);
  EXPECT_EQ(ids1.size(), 0);

  // Case: Copy from vector.
  vector<FrameId> frames{FrameId::get_new_id(), FrameId::get_new_id(),
                         FrameId::get_new_id(), FrameId::get_new_id()};
  const int frame_count = static_cast<int>(frames.size());
  FrameIdVector ids2(source_id, frames);
  EXPECT_EQ(ids2.get_source_id(), source_id);
  EXPECT_EQ(ids2.size(), frame_count);
  // Confirms successful copy.
  for (int i = 0; i < frame_count; ++i) {
    EXPECT_EQ(ids2.get_frame_id(i), frames[i]);
  }
}

// Tests the range iterators.
GTEST_TEST(FrameIdVector, RangeIteration) {
  vector<FrameId> frames{FrameId::get_new_id(), FrameId::get_new_id(),
                         FrameId::get_new_id(), FrameId::get_new_id()};
  FrameIdVector ids(SourceId::get_new_id(), frames);
  int i = 0;
  for (auto id : ids) {
    EXPECT_EQ(id, frames[i++]);
  }
  EXPECT_EQ(i, static_cast<int>(frames.size()));
}

// Tests conditions where input vector of ids contain duplicates.
GTEST_TEST(FrameIdVector, ConstructorWithDuplicates) {
  SourceId source_id = SourceId::get_new_id();
  vector<FrameId> frames{FrameId::get_new_id(), FrameId::get_new_id(),
                         FrameId::get_new_id(), FrameId::get_new_id()};
  frames.push_back(frames[0]);

  // Case: Construct by copying frames.
  EXPECT_ERROR_MESSAGE(
      FrameIdVector(source_id, frames), std::logic_error,
      "Input vector of frame ids contains duplicates.");

  // Case: Construct by moving frames.
  EXPECT_ERROR_MESSAGE(
      FrameIdVector(source_id, move(frames)), std::logic_error,
      "Input vector of frame ids contains duplicates.");
}

// Tests the functionality for adding single frames to the set.
GTEST_TEST(FrameIdVector, AddingFramesSingle) {
  FrameIdVector ids(SourceId::get_new_id());
  // Do *not* re-order these tests; the logic depends on the sequence.
  // Case: Add single to empty.
  FrameId f0 = FrameId::get_new_id();
  EXPECT_NO_THROW(ids.AddFrameId(f0));
  EXPECT_EQ(ids.size(), 1);
  EXPECT_EQ(ids.get_frame_id(0), f0);

  // Case: Add single to non-empty (unique).
  FrameId f1 = FrameId::get_new_id();
  EXPECT_NO_THROW(ids.AddFrameId(f1));
  EXPECT_EQ(ids.size(), 2);
  EXPECT_EQ(ids.get_frame_id(1), f1);

  // Case: Add single to non-empty (not unique).
  EXPECT_ERROR_MESSAGE(ids.AddFrameId(f0), std::logic_error,
                       "Id vector already contains frame id: \\d+.");
}

// Tests the functionality for adding multiple frames to the set.
GTEST_TEST(FrameIdVector, AddingFramesMultiple) {
  vector<FrameId> unique1{FrameId::get_new_id(), FrameId::get_new_id(),
                          FrameId::get_new_id(), FrameId::get_new_id()};
  vector<FrameId> unique2{FrameId::get_new_id(), FrameId::get_new_id(),
                          FrameId::get_new_id(), FrameId::get_new_id()};
  vector<FrameId> duplicate{unique1[0]};
  FrameIdVector ids(SourceId::get_new_id());
  // Do *not* re-order these tests; the logic depends on the sequence.

  // Case: Add multiple to empty (all unique).
  EXPECT_NO_THROW(ids.AddFrameIds(unique1));
  for (int i = 0; i < static_cast<int>(unique1.size()); ++i) {
    EXPECT_EQ(ids.get_frame_id(i), unique1[i]);
  }

  // Case: Add multiple to non-empty (unique result).
  EXPECT_NO_THROW(ids.AddFrameIds(unique2));
  int i = 0;
  for (; i < static_cast<int>(unique1.size()); ++i) {
    EXPECT_EQ(ids.get_frame_id(i), unique1[i]);
  }
  for (int j = 0; i < static_cast<int>(unique1.size() + unique2.size());
       ++i, ++j) {
    EXPECT_EQ(ids.get_frame_id(i), unique2[j]);
  }

  // Case: Add multiple to non-empty (non-unique result).
  EXPECT_ERROR_MESSAGE(ids.AddFrameIds(duplicate), std::logic_error,
                       "Id vector already contains frame id: \\d+.");

  // Case: Add vector of ids that do not duplicate previous contents but
  // contains duplicate values.
  FrameId new_id = FrameId::get_new_id();
  vector<FrameId> redundant{new_id, new_id};
  EXPECT_ERROR_MESSAGE(
      ids.AddFrameIds(redundant), std::logic_error,
      "Input vector of frame ids contains duplicates.");
}

// Tests the functionality that tries to report the index of the desired frame.
GTEST_TEST(FrameIdVector, FrameLookup) {
  vector<FrameId> frames{FrameId::get_new_id(), FrameId::get_new_id(),
                         FrameId::get_new_id(), FrameId::get_new_id()};
  FrameIdVector ids(SourceId::get_new_id(), frames);
  int index = -1;
  int expected_index = 2;
  EXPECT_NO_THROW(index = ids.GetIndex(frames[expected_index]));
  EXPECT_EQ(index, expected_index);

  EXPECT_ERROR_MESSAGE(ids.GetIndex(FrameId::get_new_id()), std::logic_error,
                       "The given frame id \\(\\d+\\) is not in the set.");
}

}  // namespace
}  // namespace geometry
}  // namespace drake
