#pragma once

#include "drake/automotive/maliput/api/intersection.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace api {

/// An abstract interface for providing the mapping from Intersection::Id to
/// Intersection.
class IntersectionBook {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IntersectionBook);

  virtual ~IntersectionBook() = default;

  /// Gets the specified Intersection. Returns nullptr if @p id is unrecognized.
  /// Otherwise, the returned pointer is guaranteed to remain valid throughout
  /// the lifetime of this IntersectionBook's instance.
  Intersection* GetIntersection(const Intersection::Id& id) {
    return DoGetIntersection(id);
  }

 protected:
  IntersectionBook() = default;

 private:
  virtual Intersection* DoGetIntersection(const Intersection::Id& id) = 0;
};

}  // namespace api
}  // namespace maliput
}  // namespace drake
