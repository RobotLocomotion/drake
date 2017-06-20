#include "robotlocomotion/point_t.hpp"
#include "drake/common/eigen_types.h"
#include "drake/lcm/translator_base.h"

namespace drake {
namespace lcm {
namespace robotlocomotion_translators {

/**
 * A translator between Vector3<T> and robotlocomotion::point_t.
 */
template <typename T>
class PointTranslator
    : public TranslatorBase<Vector3<T>, robotlocomotion::point_t> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PointTranslator)

  PointTranslator() {}

  const Vector3<T>& get_default_data() const override { return default_data_; }

  const robotlocomotion::point_t& get_default_msg() const override {
    return default_msg_;
  }

  /**
   * Decodes @p msg into @p data. @p data cannot be nullptr.
   */
  void Decode(const robotlocomotion::point_t& msg,
              Vector3<T>* data) const override {
    (*data)[0] = static_cast<T>(msg.x);
    (*data)[1] = static_cast<T>(msg.y);
    (*data)[2] = static_cast<T>(msg.z);
  }

  /**
   * Encodes @p data into @p msg. @p msg cannot be nullptr.
   */
  void Encode(const Vector3<T>& data,
              robotlocomotion::point_t* msg) const override {
    msg->x = static_cast<double>(data[0]);
    msg->y = static_cast<double>(data[1]);
    msg->z = static_cast<double>(data[2]);
  }

 private:
  robotlocomotion::point_t default_msg_{};
  Vector3<T> default_data_{Vector3<T>::Zero()};
};

}  // namespace robotlocomotion_translators
}  // namespace lcm
}  // namespace drake
