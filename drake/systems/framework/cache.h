#pragma once

namespace drake {
namespace systems {

// This is a placeholder.
// TODO(david-german-tri): Add actual functionality.
template <typename T>
class Cache {
 public:
  Cache() {}
  virtual ~Cache() {}

  // Cache objects are copyable with the default copy constructor.
  Cache(const Cache& other) = default;
  Cache& operator=(const Cache& other) = default;

 private:
  // Cache objects are not moveable.
  Cache(Cache&& other) = delete;
  Cache& operator=(Cache&& other) = delete;
};

}  // namespace systems
}  // namesapce drake
