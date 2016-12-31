#pragma once

#include <cmath>
#include <limits>

namespace drake {
namespace systems {
namespace sensors {

/// Holds a DepthSensor's specifications.
///
/// @see DepthSensor.
class DepthSensorSpecification {
 public:
  /// Constructs a %DepthSensorSpecification.
  ///
  /// @param[in] min_yaw The minimum horizontal scan angle in the sensor's
  /// base frame. The horizontal scan angle is about the +Z axis. A zero angle
  /// is the +X axis.
  ///
  /// @param[in] max_yaw The maximum horizontal scan angle in the sensor's
  /// base frame.
  ///
  /// @param[in] min_pitch The minimum vertical scan angle in the sensor's base
  /// frame.
  ///
  /// @param[in] max_pitch The maximum vertical scan angle in the sensor's base
  /// frame.
  ///
  /// @param[in] num_yaw_values The number of yaw values at which depth
  /// measurements are taken. These measurements are evenly spread between
  /// @p min_yaw and @p max_yaw. This value must be greater than or equal to
  /// 1, which occurs when @p min_yaw equals @p max_yaw.
  ///
  /// @param[in] num_pitch_values The number of yaw values at which depth
  /// measurements are taken. These measurements are evenly spread between
  /// @p min_pitch and @p max_pitch. This value must be greater than or equal to
  /// 1, which occurs when @p min_pitch equals @p max_pitch.
  ///
  /// @param[in] min_range The minimum sensing range.
  ///
  /// @param[in] max_range The maximum sensing range.
  DepthSensorSpecification(double min_yaw, double max_yaw, double min_pitch,
                           double max_pitch, int num_yaw_values,
                           int num_pitch_values, double min_range,
                           double max_range);

  /// @name Accessors to manually specified parameters.
  ///
  /// The following accessors are for parameter values that are manually
  /// specified. They appear as input parameters of the constructor, and
  /// mutator methods in this class.
  ///@{
  double min_yaw() const { return min_yaw_; }
  double max_yaw() const { return max_yaw_; }
  double min_pitch() const { return min_pitch_; }
  double max_pitch() const { return max_pitch_; }
  int num_yaw_values() const { return num_yaw_values_; }
  int num_pitch_values() const { return num_pitch_values_; }
  int num_depth_readings() const { return num_depth_readings_; }
  double min_range() const { return min_range_; }
  double max_range() const { return max_range_; }
  ///@}

  /// @name Accessor to automatically derived parameters.
  ///
  /// The following accessors are for values that are derived based on the
  /// manually specified parameters.
  ///@{
  double yaw_increment() const { return yaw_increment_; }
  double pitch_increment() const { return pitch_increment_; }
  ///@}

  void set_min_yaw(double min_yaw);
  void set_max_yaw(double max_yaw);
  void set_num_yaw_values(double num_yaw_values);
  void set_num_pitch_values(double num_yaw_values);

  /// @name Convenient Instantiations of DepthSensorSpecification.
  ///
  /// The following accessors provide instantiations of frequently used
  /// DepthSensorSpecification objects that can be used directly or as templates
  /// for customization.
  ///@{

  /// Returns a DepthSensorSpecification that covers octant 1 of the sensor's
  /// base frame. It contains the following specifications:
  ///
  ///  - min_yaw = 0
  ///  - max_yaw = M_PI / 2
  ///  - min_pitch = 0
  ///  - max_pitch = M_PI / 2
  ///  - num_yaw_values = 10
  ///  - num_pitch_values = 5
  ///  - min_range = 0
  ///  - max_range = 1
  ///
  static const DepthSensorSpecification& get_octant_1_spec() {
    static const DepthSensorSpecification spec(0,         // min_yaw
                                               M_PI / 2,  // max_yaw
                                               0,         // min_pitch
                                               M_PI / 2,  // max_pitch
                                               10,        // num_yaw_values
                                               5,         // num_pitch_values
                                               0,         // min_range
                                               1);        // max_range
    return spec;
  }

  /// Returns a DepthSensorSpecification that covers the sensor's base frame's
  /// X/Y plane. It contains the following specifications:
  ///
  ///  - min_yaw = -M_PI
  ///  - max_yaw = M_PI
  ///  - min_pitch = 0
  ///  - max_pitch = 0
  ///  - num_yaw_values = 50
  ///  - num_pitch_values = 1
  ///  - min_range = 0
  ///  - max_range = 1
  ///
  static const DepthSensorSpecification& get_xy_planar_spec() {
    static const DepthSensorSpecification spec(-M_PI,  // min_yaw
                                               M_PI,   // max_yaw
                                               0,      // min_pitch
                                               0,      // max_pitch
                                               50,     // num_yaw_values
                                               1,      // num_pitch_values
                                               0,      // min_range
                                               1);     // max_range
    return spec;
  }

  /// Returns a DepthSensorSpecification that covers the sensor's base frame's
  /// X/Z plane. It contains the following specifications:
  ///
  ///  - min_yaw = 0
  ///  - max_yaw = 0
  ///  - min_pitch = -M_PI / 2
  ///  - max_pitch = M_PI / 2
  ///  - num_yaw_values = 1
  ///  - num_pitch_values = 50
  ///  - min_range = 0
  ///  - max_range = 1
  ///
  static const DepthSensorSpecification& get_xz_planar_spec() {
    static const DepthSensorSpecification spec(0,          // min_yaw
                                               0,          // max_yaw
                                               -M_PI / 2,  // min_pitch
                                               M_PI / 2,   // max_pitch
                                               1,          // num_yaw_values
                                               50,         // num_pitch_values
                                               0,          // min_range
                                               1);         // max_range
    return spec;
  }

  /// Returns a DepthSensorSpecification that covers all 8 octants of the
  /// sensor's base frame. The sensed region is in the shape of a hollow
  /// sphere with a radius between min_range and max_range. It contains the
  /// following specifications:
  ///
  ///  - min_yaw = -M_PI
  ///  - max_yaw = M_PI
  ///  - min_pitch = -M_PI / 2
  ///  - max_pitch = M_PI / 2
  ///  - num_yaw_values = 50
  ///  - num_pitch_values = 50
  ///  - min_range = 0
  ///  - max_range = 1
  ///
  /// Note that since min_range is 0, the sensed sphere is actually solid.
  static const DepthSensorSpecification& get_xyz_spherical_spec() {
    static const DepthSensorSpecification spec(-M_PI,      // min_yaw
                                               M_PI,       // max_yaw
                                               -M_PI / 2,  // min_pitch
                                               M_PI / 2,   // max_pitch
                                               50,         // num_yaw_values
                                               50,         // num_pitch_values
                                               0,          // min_range
                                               1);         // max_range
    return spec;
  }

  /// Returns a DepthSensorSpecification that consists of a single depth
  /// measurement along the sensor's base frame's +X axis between 1 meter and 2
  /// meters. It contains the following specifications:
  ///
  ///  - min_yaw = 0
  ///  - max_yaw = 0
  ///  - min_pitch = 0
  ///  - max_pitch = 0
  ///  - num_yaw_values = 1
  ///  - num_pitch_values = 1
  ///  - min_range = 1
  ///  - max_range = 2
  ///
  static const DepthSensorSpecification& get_x_linear_spec() {
    static const DepthSensorSpecification spec(0,   // min_yaw
                                               0,   // max_yaw
                                               0,   // min_pitch
                                               0,   // max_pitch
                                               1,   // num_yaw_values
                                               1,   // num_pitch_values
                                               1,   // min_range
                                               2);  // max_range
    return spec;
  }
  ///@}
 private:
  void update_num_depth_readings();
  void update_yaw_increment();
  void update_pitch_increment();

  double min_yaw_{};
  double max_yaw_{};
  double min_pitch_{};
  double max_pitch_{};
  int num_yaw_values_{};
  int num_pitch_values_{};
  int num_depth_readings_{};
  double min_range_{0};
  double max_range_{std::numeric_limits<double>::max()};

  // The following variables are computed based on the above variables.
  double yaw_increment_{};
  double pitch_increment_{};
};

inline bool operator==(const DepthSensorSpecification& lhs,
                       const DepthSensorSpecification& rhs) {
  return lhs.min_yaw() == rhs.min_yaw() && lhs.max_yaw() == rhs.max_yaw() &&
         lhs.min_pitch() == rhs.min_pitch() &&
         lhs.max_pitch() == rhs.max_pitch() &&
         lhs.num_yaw_values() == rhs.num_yaw_values() &&
         lhs.num_pitch_values() == rhs.num_pitch_values() &&
         lhs.num_depth_readings() == rhs.num_depth_readings() &&
         lhs.min_range() == rhs.min_range() &&
         lhs.max_range() == rhs.max_range() &&
         lhs.yaw_increment() == rhs.yaw_increment() &&
         lhs.pitch_increment() == rhs.pitch_increment();
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
