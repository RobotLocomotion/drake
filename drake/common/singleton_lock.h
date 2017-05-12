#pragma once

#include <memory>
#include <mutex>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/never_destroyed.h"

namespace drake {

/*
 * @brief Thread-safe, global-safe implementation for acquiring a shared
 * resources necessary, and releasing the resource when able. Use this for
 * obtaining licenses for solvers, where network latency may be a significant
 * hit.
 * @tparam T Class of the resource. Must be default-constructible.
 * @tparam Parent Optional class of lock owner. This is meant to make a unique
 * specialization, such that you can use multiple disjoint SingletonLock on T
 * (for whatever reason).
 *
 * @note No mutex should be necessary for accessing the resource, as this
 * is handled by the lifetime of the lock.
 * That is, if you are losing the resource through
 * this class, there is either a bug in this code or your code.
 */
template <typename T, typename Parent = void>
class SingletonLock {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SingletonLock)

  SingletonLock()
    : instance_(manager().AcquireIfNeeded()) {}

  ~SingletonLock() {
    instance_.reset();
    manager().ReleaseIfAble();
  }

  T& instance() {
    return *instance_;
  }

  const T& instance() const {
    return *instance_;
  }

  int use_count() const {
    // Return number of references that are external to the manager.
    return instance_.use_count() - 1;
  }

 private:
  std::shared_ptr<T> instance_;

  /*
   * Manager for the global reference.
   */
  class Manager {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Manager)
    Manager() = default;

    /*
     * Acquire the resource if there are no existing references.
     * Return the singleton reference.
     */
    std::shared_ptr<T> AcquireIfNeeded() {
      // Perform a locked acquisition, given that this is effectively a global.
      std::lock_guard<std::mutex> lock(mutex_);
      if (!singleton_) {
        // No prior references.
        singleton_.reset(new T());
      }
      return singleton_;
    }

    /*
     * Release the resource only if there are no external references to the
     * singleton.
     */
    void ReleaseIfAble() {
      std::lock_guard<std::mutex> lock(mutex_);
      // This should never be called on an empty resource.
      DRAKE_DEMAND(singleton_ != nullptr);
      if (singleton_.use_count() == 1) {
        // No external references to singleton. Release resource.
        singleton_.reset();
      }
    }

   private:
    std::shared_ptr<T> singleton_;
    // This mutex is ONLY intended to handle acquiring / releasing the
    // resource. It is not meant for accessing the resource.
    std::mutex mutex_;
  };

  /*
   * Ensure that the mutex is not destructed while a
   * a lock is being constructed / destructed.
   * This also prevents ~T() from being called by ~shared_ptr<T().
   *
   * @note Singleton is managed via static method since C++ requires
   * explicit specialization to define a static member of a nested
   * type in a templated class.
   */
  static Manager& manager() {
    // This behavior can be changed if needed.
    static never_destroyed<Manager> manager;
    return manager.access();
  }
};

}  // namespace drake
