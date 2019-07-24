#pragma once

#include <vector>

#include "drake/automotive/deprecated.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace geometry_base {


/// geometry_base's implementation of api::LaneEndSet.
class DRAKE_DEPRECATED_AUTOMOTIVE
    LaneEndSet : public api::LaneEndSet {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LaneEndSet);

  /// Constructs an empty LaneEndSet.
  LaneEndSet() = default;

  /// Adds an api::LaneEnd to the set.
  ///
  /// @throws std::exception if `end.lane` is nullptr.
  void Add(const api::LaneEnd& end) {
    // TODO(maddog@tri.global)  This assertion belongs in LaneEnd itself.
    DRAKE_THROW_UNLESS(end.lane != nullptr);
    ends_.push_back(end);
  }

  ~LaneEndSet() override = default;

 private:
  int do_size() const override { return ends_.size(); }

  const api::LaneEnd&
  do_get(int index) const override { return ends_.at(index); }

  std::vector<api::LaneEnd> ends_;
};


}  // namespace geometry_base
}  // namespace maliput
}  // namespace drake
