#include "drake/geometry/frame_id_vector.h"

#include <string>
#include <unordered_set>
#include <utility>

namespace drake {
namespace geometry {

using std::vector;

FrameIdVector::FrameIdVector(SourceId source_id) : source_id_(source_id) {}

FrameIdVector::FrameIdVector(SourceId source_id, const vector<FrameId>& ids)
    : source_id_(source_id) {
  AddFrameIds(ids);
}

int FrameIdVector::GetIndex(FrameId frame_id) const {
  auto iter = id_index_map_.find(frame_id);
  if (iter != id_index_map_.end()) {
    return iter->second;
  }
  using std::to_string;
  throw std::logic_error(
      "The given frame id (" + to_string(frame_id) + ") is not in the set.");
}

void FrameIdVector::AddFrameId(FrameId frame_id) {
  ThrowIfContains(frame_id);
  int index = static_cast<int>(id_index_map_.size());
  id_index_map_[frame_id] = index;
  index_id_map_.push_back(frame_id);
}

void FrameIdVector::AddFrameIds(const vector<FrameId>& ids) {
  ThrowIfContainsDuplicates(ids);
  id_index_map_.reserve(id_index_map_.size() + ids.size());
  for (auto id : ids) {
    AddFrameId(id);
  }
}

void FrameIdVector::ThrowIfContainsDuplicates(
    const std::vector<FrameId>& frame_ids) {
  std::unordered_set<FrameId> unique_ids{frame_ids.begin(), frame_ids.end()};
  if (unique_ids.size() != frame_ids.size()) {
    throw std::logic_error("Input vector of frame ids contains duplicates.");
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
