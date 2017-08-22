#include "drake/geometry/frame_id_vector.h"

#include <string>
#include <unordered_set>
#include <utility>

namespace drake {
namespace geometry {

using std::move;
using std::vector;

FrameIdVector::FrameIdVector(SourceId source_id) : source_id_(source_id) {}

FrameIdVector::FrameIdVector(SourceId source_id, const vector<FrameId>& ids)
    : source_id_(source_id) {
  AddFrameIds(ids);
}

int FrameIdVector::GetIndex(FrameId frame_id) const {
  auto itr = id_index_map_.find(frame_id);
  if (itr != id_index_map_.end()) {
    return itr->second;
  }
  using std::to_string;
  throw std::logic_error(
      "The given frame id (" + to_string(frame_id) + ") is not in the set.");
}

int FrameIdVector::AddFrameId(FrameId frame_id) {
  DRAKE_ASSERT_VOID(ThrowIfContains(frame_id));
  int index = static_cast<int>(id_index_map_.size());
  id_index_map_[frame_id] = index;
  index_id_map_.push_back(frame_id);
  return index + 1;
}

int FrameIdVector::AddFrameIds(const vector<FrameId>& ids) {
  DRAKE_ASSERT_VOID(ThrowIfContainsDuplicates(ids));
  DRAKE_ASSERT_VOID(ThrowIfContains(ids));
  int start_index = static_cast<int>(id_index_map_.size());
  index_id_map_.resize(start_index + ids.size());
  for (auto id : ids) {
    index_id_map_[start_index] = id;
    id_index_map_[id] = start_index++;
  }
  return start_index;
}

void FrameIdVector::ThrowIfContainsDuplicates(
    const std::vector<FrameId>& frame_ids) {
  std::unordered_set<FrameId> unique_ids{frame_ids.begin(), frame_ids.end()};
  if (unique_ids.size() != frame_ids.size()) {
    throw std::logic_error("Input vector of frame ids contains duplicates.");
  }
}

void FrameIdVector::ThrowIfContains(const std::vector<FrameId>& frame_ids) {
  using std::to_string;
  for (auto f_id : frame_ids) {
    if (id_index_map_.find(f_id) != id_index_map_.end()) {
      throw std::logic_error("The frame id vector contains duplicate frame"
                                " ids, including, at least, " +
          to_string(f_id) + ".");
    }
  }
}

void FrameIdVector::ThrowIfContains(FrameId frame_id) {
  if (id_index_map_.find(frame_id) != id_index_map_.end()) {
    using std::to_string;
    throw std::logic_error("Id vector already contains frame id: " +
        to_string(frame_id) + ".");
  }
}

}  // namespace geometry
}  // namespace drake
