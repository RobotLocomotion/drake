#include "robotlocomotion/quaternion_t.hpp"
#include "drake/common/eigen_types.h"
#include "drake/lcm/translator_base.h"

namespace drake {
namespace lcm {
namespace robotlocomotion_translators {

/**
 * A translator between Quaternion<T> and robotlocomotion::quaternion_t.
 */
template <typename T>
class QuaternionTranslator
    : public TranslatorBase<Quaternion<T>, robotlocomotion::quaternion_t> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QuaternionTranslator)

  QuaternionTranslator() {
    default_msg_.w = 1;
    default_msg_.x = default_msg_.y = default_msg_.z = 0;
  }

  const Quaternion<T>& get_default_data() const override {
    return default_data_;
  }

  const robotlocomotion::quaternion_t& get_default_msg() const override {
    return default_msg_;
  }

  /**
   * Decodes @p msg into @p data. Note that @p data will be normalized.
   * @p data cannot be nullptr.
   */
  void Decode(const robotlocomotion::quaternion_t& msg,
              Quaternion<T>* data) const override {
    *data = Quaternion<T>(msg.w, msg.x, msg.y, msg.z);
    data->normalize();
  }

  /**
   * Encodes @p data into @p msg. @p msg cannot be nullptr.
   */
  void Encode(const Quaternion<T>& data,
              robotlocomotion::quaternion_t* msg) const override {
    msg->w = data.w();
    msg->x = data.x();
    msg->y = data.y();
    msg->z = data.z();
  }

 private:
  Quaternion<T> default_data_{Quaternion<T>::Identity()};
  robotlocomotion::quaternion_t default_msg_;
};

}  // namespace robotlocomotion_translators
}  // namespace lcm
}  // namespace drake
