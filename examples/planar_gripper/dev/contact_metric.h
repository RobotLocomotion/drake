#pragma once

#include "drake/examples/planar_gripper/dev/contact_mode.h"

namespace drake {
namespace examples {
namespace planar_gripper {

// An abstract class to implement metrics over contact modes
template <typename T>
class ContactMetric {
 public:
  virtual T eval(ContactMode*, ContactMode*) = 0;
  virtual ~ContactMetric(){};
};

template <typename T>
class MapBasedContactMetric : public ContactMetric<T> {
 public:
  MapBasedContactMetric() : default_value_(0) {}
  MapBasedContactMetric(
      std::map<std::pair<ContactMode*, ContactMode*>, T> metric)
      : metric_(metric), default_value_(0) {}

  T eval(ContactMode* cm1, ContactMode* cm2) {
    if (metric_.find({cm1, cm2}) != metric_.end()) {
      return metric_[{cm1, cm2}];
    }
    return default_value_;
  }

  void add_value(ContactMode* cm1, ContactMode* cm2, T value) {
    metric_[std::make_pair(cm1, cm2)] = value;
  }

  T get_default_value() { return default_value_; }
  void set_default_value(T value) { default_value_ = value; }

 private:
  std::map<std::pair<ContactMode*, ContactMode*>, T> metric_;
  T default_value_;
};

}  // namespace planar_gripper
}  // namespace examples
}  // namespace drake
