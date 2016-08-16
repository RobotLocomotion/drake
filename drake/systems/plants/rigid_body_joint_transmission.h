#pragma once

/**
 * Defines a joint transmission, such that the joint position should satisfy the
 * constraint
 * joint1_position = joint2_position * multiplier + offset.
 */
class DRAKERBM_EXPORT RigidBodyJointTransmission {
 public:
  /** Constructs a joint transmission by specifying the position numbers of the
   two
   joints, together with the multiplier

   @param[in] joint1_name  A string, the name of joint 1.
   @param[in] joint2_name  A string, the name of joint 2.
   @param[in] multiplier  A double, the ratio between the values of
   (joint1-offset)/joint2
   @param[in] offset  A double, offset = joint1_value-joint2_value*multiplier.
   */
  RigidBodyJointTransmission(const std::string& joint1_name,
                             const std::string& joint2_name, double multiplier,
                             double offset, int model_instance_id)
      : joint1_name_(joint1_name),
        joint2_name_(joint2_name),
        multiplier_(multiplier),
        offset_(offset),
        model_instance_id_(model_instance_id) {
    DRAKE_ASSERT(multiplier > 0);
  }
  std::string GetJoint1Name() const { return joint1_name_; }
  std::string GetJoint2Name() const { return joint2_name_; }
  double GetMultiplier() const { return multiplier_; }
  double GetOffset() const { return offset_; }
  int GetModelInstanceID() const { return model_instance_id_; }

 private:
  std::string joint1_name_, joint2_name_;
  double multiplier_;
  double offset_;
  int model_instance_id_;
};
