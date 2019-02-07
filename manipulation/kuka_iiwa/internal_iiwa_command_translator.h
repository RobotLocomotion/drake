#pragma once

/// @file
/// This file is deprecated and will be removed after 2019-03-01.

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {
namespace internal {

/**
 * A vectorized representation of lcmt_iiwa_command.
 */
template <typename T>
class IiwaCommand : public systems::BasicVector<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaCommand)

  static constexpr double kUnitializedTime = 0.0;

  // TODO(jwnimmer-tri) Remove this class after 2019-03-01.
  // There is no replacement -- these objects should never be needed anymore.
  /**
   * The dimension of this will be 2 * @p num_joints + 1, which are timestamp,
   * position and torque per each joint.
   */
  DRAKE_DEPRECATED("This class will be removed after 2019-03-01")
  explicit IiwaCommand(int num_joints);

  T utime() const;

  Eigen::VectorBlock<const VectorX<T>> joint_position() const;

  Eigen::VectorBlock<const VectorX<T>> joint_torque() const;

  void set_utime(T utime);

  /**
   * @throws if the dimension of @p q does not match num_joints at construction
   * time.
   */
  void set_joint_position(const VectorX<T>& q);

  /**
   * @throws if the dimension of @p q does not match num_joints at construction
   * time.
   */
  void set_joint_torque(const VectorX<T>& torque);

 private:
  IiwaCommand<T>* DoClone() const override {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    return new IiwaCommand<T>(num_joints_);
#pragma GCC diagnostic pop
  }

  const int num_joints_;
};

/**
 * A translator between the LCM message type lcmt_iiwa_command and its
 * vectorized representation, IiwaCommand<double>. This is intended to be used
 * with systems::lcm::LcmPublisherSystem and systems::lcm::LcmSubscriberSystem.
 */
class IiwaCommandTranslator : public systems::lcm::LcmAndVectorBaseTranslator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaCommandTranslator)

  // TODO(jwnimmer-tri) Remove this class after 2019-03-01.
  // There is no replacement -- these objects should never be needed anymore.
  /**
   * Constructs a IiwaCommandTranslator.
   * @param num_joints Number of joints of the IIWA command.
   */
  DRAKE_DEPRECATED("This class will be removed after 2019-03-01")
  explicit IiwaCommandTranslator(int num_joints = kIiwaArmNumJoints);

  std::unique_ptr<systems::BasicVector<double>> AllocateOutputVector()
      const override;

  /**
   * Translates @p lcm_message_bytes into @p vector_base. Assumes that the
   * size of `joint_position` field in the decoded messages matches the
   * declared number of joints at construction time. If the decoded
   * `joint_torque` field is empty, the torque part of @p vector_base will
   * be filled by zeros.
   * Throws if
   * - @p lcm_message_bytes cannot be decoded as a lcmt_iiwa_command.
   * - @p vector_base is not a IiwaCommand<double>.
   * - The decoded `joint_position` in @p lcm_message_bytes has a different
   *   size.
   * - The decoded `joint_torque` in @p lcm_message_bytes has a different size.
   */
  void Deserialize(const void* lcm_message_bytes, int lcm_message_length,
                   systems::VectorBase<double>* vector_base) const override;

  /**
   * Not implemented.
   * @throws std::runtime_error.
   */
  void Serialize(double time, const systems::VectorBase<double>& vector_base,
                 std::vector<uint8_t>* lcm_message_bytes) const override;

 private:
  const int num_joints_;
};

}  // namespace internal
}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
