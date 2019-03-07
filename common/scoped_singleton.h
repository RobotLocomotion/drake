#pragma once

#include <memory>
#include <mutex>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/text_logging.h"

namespace drake {

/**
 * Provides thread-safe, global-safe access to a shared resource. When
 * all references are gone, the resource will be freed due to using a weak_ptr.
 * @tparam T Class of the resource. Must be default-constructible.
 * @tparam Unique Optional class, meant to make a unique specialization, such
 * that you can have multiple singletons of T if necessary.
 *
 * @note An example application is obtaining license for multiple disjoint
 * solver objects, where acquiring a license requires network communication,
 * and the solver is under an interface where you cannot explicitly pass the
 * license resource to the solver without violating encapsulation.
 */
template <typename T, typename Unique = void>
std::shared_ptr<T> GetScopedSingleton() {
  // Confine implementation to a class.
  class Singleton {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Singleton)

    Singleton() {}

    /*
     * Acquire a reference to resource if no other instance exists.
     * Otherwise, return a shared reference to the resource.
     */
    std::shared_ptr<T> Acquire() {
      // We must never create more than one instance of a T at a time, since it
      // is supposed to be a singleton.  Thus, we need at least the make_shared
      // to appear in a critical section.  Rather than worrying about a double-
      // checked locking pattern, we'll just hold the lock for the entire
      // method for simplicity.
      std::lock_guard<std::mutex> lock(mutex_);
      std::shared_ptr<T> result;

      // Attempt to promote the weak_ptr to a shared_ptr.  If the singleton
      // already existed, this will extend its lifetime to our return value.
      result = weak_ref_.lock();
      if (!result) {
        // The singleton does not exist.  Since we hold the exclusive lock on
        // weak_ref_, we know that its safe for us to create the singleton now.
        result = std::make_shared<T>();

        // Store the weak reference to the result, so that other callers will
        // be able to use it, as long as our caller keeps it alive.
        weak_ref_ = result;
      }

      DRAKE_DEMAND(result.get() != nullptr);
      return result;
    }

   private:
    // The mutex guards all access and changes to weak_ref_, but does not guard
    // the use of the T instance that weak_ref_ points to.
    std::mutex mutex_;
    std::weak_ptr<T> weak_ref_;
  };
  // Allocate singleton as a static function local to control initialization.
  static never_destroyed<Singleton> singleton;
  return singleton.access().Acquire();
}

}  // namespace drake
