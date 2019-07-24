#pragma once

#include <map>
#include <utility>

#include "drake/examples/planar_gripper/dev/contact_mode.h"

namespace drake {
namespace examples {
namespace planar_gripper {

/// ContactMetric is an abstract class to implement metrics over contact modes.
/// It specifies an 'Eval' function that can be used in search algorithms to get
/// the distance between two contact modes in some metric space
template <typename T>
class ContactMetric {
 public:
  virtual T Eval(const ContactMode& from_mode,
                 const ContactMode& to_mode) const = 0;
  virtual ~ContactMetric() {}
};

/// MapBasedContactMetric is an implementation of ContactMetric. It stores the
/// distance between two contact modes by using a 'map' data structure. The
/// distance between nodes can be set using SetValue. If a value hasn't been
/// set, the Eval function returns the default_value
template <typename T>
class MapBasedContactMetric : public ContactMetric<T> {
 public:
  MapBasedContactMetric() : default_value_(0) {}
  MapBasedContactMetric(
      std::map<std::pair<ContactMode::Id, ContactMode::Id>, T> metric)
      : metric_(std::move(metric)), default_value_(0) {}

  T Eval(const ContactMode& from_mode, const ContactMode& to_mode) const {
    auto pair = std::make_pair(from_mode.get_id(), to_mode.get_id());
    auto value = metric_.find(pair);
    if (value != metric_.end()) {
      return (*value).second;
    }
    return default_value_;
  }

  void SetValue(const ContactMode& from_mode, const ContactMode& to_mode,
                T value) {
    metric_[std::make_pair(from_mode.get_id(), to_mode.get_id())] = value;
  }

  T get_default_value() const { return default_value_; }
  void set_default_value(const T value) { default_value_ = value; }

 private:
  std::map<std::pair<ContactMode::Id, ContactMode::Id>, T> metric_;
  T default_value_;
};

}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
