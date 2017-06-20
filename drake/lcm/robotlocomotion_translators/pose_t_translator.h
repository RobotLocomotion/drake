#include "robotlocomotion/pose_t.hpp"
#include "drake/common/eigen_types.h"
#include "drake/lcm/robotlocomotion_translators/point_t_translator.h"
#include "drake/lcm/robotlocomotion_translators/quaternion_t_translator.h"
#include "drake/lcm/translator_base.h"

namespace drake {
namespace lcm {
namespace robotlocomotion_translators {

/**
 * A translator between Isometry3<T> and robotlocomotion::pose_t.
 */
template <typename T>
class PoseTranslator
    : public drake::lcm::TranslatorBase<Isometry3<T>, robotlocomotion::pose_t> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PoseTranslator)

  PoseTranslator() {
    default_msg_.position = point_translator_.get_default_msg();
    default_msg_.orientation = quaternion_translator_.get_default_msg();
  }

  const Isometry3<T>& get_default_data() const override {
    return default_data_;
  }

  const robotlocomotion::pose_t& get_default_msg() const override {
    return default_msg_;
  }

  /**
   * Decodes @p msg into @p data. Note that the quaternion part in @p msg is
   * normalized first.
   * @p data cannot be nullptr.
   */
  void Decode(const robotlocomotion::pose_t& msg,
              Isometry3<T>* data) const override {
    Quaternion<T> tmp_rot;
    Vector3<T> tmp_vec;
    point_translator_.Decode(msg.position, &tmp_vec);
    quaternion_translator_.Decode(msg.orientation, &tmp_rot);
    data->translation() = tmp_vec;
    data->linear() = tmp_rot.toRotationMatrix();
    data->makeAffine();
  }

  /**
   * Encodes @p data into @p msg. @p msg cannot be nullptr.
   */
  void Encode(const Isometry3<T>& data,
              robotlocomotion::pose_t* msg) const override {
    point_translator_.Encode(data.translation(), &(msg->position));
    quaternion_translator_.Encode(Quaternion<T>(data.linear()),
                                  &(msg->orientation));
  }

 private:
  Isometry3<T> default_data_{Isometry3<T>::Identity()};
  robotlocomotion::pose_t default_msg_;
  const PointTranslator<T> point_translator_;
  const QuaternionTranslator<T> quaternion_translator_;
};

}  // namespace robotlocomotion_translators
}  // namespace lcm
}  // namespace drake
