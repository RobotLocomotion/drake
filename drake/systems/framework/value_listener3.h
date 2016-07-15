#pragma once

#include <algorithm>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/drakeSystemFramework_export.h"

namespace drake {
namespace systems {

/** ValueListenerInterface is an interface that dependent computations must
implement so that they can receive notifications when a prerequisite value
is modified. The prerequisite may be an independent value, like time or a
state variable, or it may itself be a computation, which computation might be
local or delivered through an input port.

This interface is a Drake-internal detail. Users should not implement it. **/
class ValueListenerInterface {
 public:
  virtual ~ValueListenerInterface() {}

  /** Mark this dependent computation "not current", and propagate that to
  downstream dependents of this listener. **/
  virtual void Invalidate() = 0;
};

/** A %ValueListenerList object maintains a list of value listeners that
have registered to receive notification of value changes from the holder of that
list. Any Context (runtime) object that can have downstream dependents should
contain one of these lists, including input and output ports, time and state
variables, and cache entries. When a value is changed, or returned with mutable
access, the `Invalidate()` method should be called on each listener. **/
class ValueListenerList {
public:
  /** Add a new listener to this list. It is a programming error if the listener
  is already on the list; we'll `abort()` at least in Debug builds. **/
  void add_listener(ValueListenerInterface* listener) {
    // This check is too expensive for Release builds.
    DRAKE_ASSERT(std::find(listeners_.begin(), listeners_.end(), listener)
                 == listeners_.end());
    listeners_.push_back(listener);
  }

  /** Remove a listener from the list. This should rarely if ever be necessary
  so we don't optimize for it: takes O(N) time. It is a programming error
  if the listener is not present on the list; we'll abort() at least in Debug
  builds. **/
  void RemoveListener(ValueListenerInterface* listener) {
    auto found = std::find(listeners_.begin(), listeners_.end(), listener);
    DRAKE_ABORT_UNLESS(found != listeners_.end());
    listeners_.erase(found);
  }

  /** Notify all dependents of this value that the value has changed. **/
  void NotifyListeners() {
    for (auto listener : listeners_) 
      listener->Invalidate();
  }
private:
  // Raw pointers to listeners; no ownership implied.
  std::vector<ValueListenerInterface*> listeners_;
};

}  // namespace systems
}  // namespace drake
