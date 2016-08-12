#pragma once

/**
 * Defines a joint transmission, such that the joint position should satisfy the
 * constraint
 * joint1_position = joint2_position * multiplier + offset.
 */
class DRAKERBM_EXPORT RigidBodyJointTransmission {
 public:
  /**
* Constructs a joint transmission by specifying the position numbers of the two
* joints, together with the multiplier
*
* @param[in] joint1_name  A string, the name of joint 1
*
* @param[in] joint2_name  A string, the name of joint 2
*
* @param[in] multiplier  A double, the ratio between the values of joint1/joint2
*
* @param[in] offset  A double, offset = joint1_value-joint2_value*multiplier
   */
  RigidBodyJointTransmission(const std::string& joint1_name,
                             const std::string& joint2_name, double multiplier,
                             double offset)
      : joint1_name_(joint1_name),
        joint2_name_(joint2_name),
        multiplier_(multiplier),
        offset_(offset) {
    DRAKE_ASSERT(multiplier > 0);
  }
  std::string getJoint1Name() const { return joint1_name_; }
  std::string getJoint2Name() const { return joint2_name_; }
  double getMultiplier() const { return multiplier_; }
  double getOffset() const { return offset_; }

 private:
  std::string joint1_name_, joint2_name_;
  double multiplier_;
  double offset_;
};
