#pragma once

#include <memory>
#include <mutex>

#include "drake/common/never_destroyed.h"

namespace drake {

/**
 * Provides thread-safe, global-safe access to a shared resource. When
 * all references are gone, the resource will be freed due to using a weak_ptr.
 * @tparam T Class of the resource. Must be default-constructible.
 * @tparam Unique Optional class of scope owner. This is meant to make a unique
 * specialization, such that you can have multiple singletons of T.
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
    Singleton() {
      // Ensure that releasing the resource is thread-safe.
      deleter_ = [this](T* ptr) {
        std::lock_guard<std::mutex> lock(mutex_);
        delete ptr;
      };
    };

    /*
     * Acquire a reference to resource if no other instance exists.
     * Otherwise, return a shared reference to the resource.
     */
    std::shared_ptr<T> Acquire() {
      // Ensure that acquiring the resource is thread-safe.
      std::lock_guard<std::mutex> lock(mutex_);
      auto instance = weak_ref_.lock();
      if (!instance) {
        instance = std::shared_ptr<T>(new T(), deleter_);
        weak_ref_ = instance;
      }
      return instance;
    }
   private:
    std::weak_ptr<T> weak_ref_;
    std::function<void(T*)> deleter_;
    std::mutex mutex_;
  };
  // Allocate singleton as a static function local to control initialization.
  static never_destroyed<Singleton> singleton;
  return singleton.access().Acquire();
}

}  // namespace drake
