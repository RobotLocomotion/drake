#pragma once

#include <cmath>
#include <limits>

namespace drake {
namespace systems {
namespace sensors {

/// Holds the specifications of a DepthSensor.
///
/// @see DepthSensor.
class DepthSensorSpecification {
 public:
  /// Constructs a %DepthSensorSpecification.
  ///
  /// @param[in] min_theta The minimum horizontal scan angle in the sensor's
  /// base frame. The horizontal scan angle is about the +Z axis. An angle of
  /// zero is the +X axis.
  ///
  /// @param[in] max_theta The maximum horizontal scan angle in the sensor's
  /// base frame.
  ///
  /// @param[in] min_phi The minimum vertical scan angle in the sensor's base
  /// frame.
  ///
  /// @param[in] max_phi The maximum vertical scan angle in the sensor's base
  /// frame.
  ///
  /// @param[in] num_theta_values The number of theta values at which depth
  /// measurements are taken. These measurements are evenly spread between
  /// @p min_theta and @p max_theta. This value must be greater than or equal to
  /// 1, which occurs when @p min_theta equals @p max_theta.
  ///
  /// @param[in] num_phi_values The number of theta values at which depth
  /// measurements are taken. These measurements are evenly spread between
  /// @p min_phi and @p max_phi. This value must be greater than or equal to 1,
  /// which occurs when @p min_phi equals @p max_phi.
  ///
  /// @param[in] min_range The minimum sensing range.
  ///
  /// @param[in] max_range The maximum sensing range.
  DepthSensorSpecification(double min_theta, double max_theta, double min_phi,
                           double max_phi, int num_theta_values,
                           int num_phi_values, double min_range,
                           double max_range);

  double min_theta() const { return min_theta_; }
  double max_theta() const { return max_theta_; }
  double min_phi() const { return min_phi_; }
  double max_phi() const { return max_phi_; }
  int num_theta_values() const { return num_theta_values_; }
  int num_phi_values() const { return num_phi_values_; }
  int num_depth_readings() const { return num_depth_readings_; }
  double min_range() const { return min_range_; }
  double max_range() const { return max_range_; }

  double theta_increment() const { return theta_increment_; }
  double phi_increment() const { return phi_increment_; }

  void set_min_theta(double min_theta);
  void set_max_theta(double max_theta);
  void set_num_theta_values(double num_theta_values);
  void set_num_phi_values(double num_theta_values);

  /// @name Convenient Instantiations of DepthSensorSpecification.
  ///
  /// The following accessors provide instantiations of frequently used
  /// DepthSensorSpecification objects that can be used directly or as templates
  /// for customization.
  ///@{

  /// Returns a DepthSensorSpecification that covers octant 1 of the sensor's
  /// base frame. It contains the following specifications:
  ///
  ///  - min_theta = 0
  ///  - max_theta = M_PI / 2
  ///  - min_phi = 0
  ///  - max_phi = M_PI / 2
  ///  - num_theta_values = 10
  ///  - num_phi_values = 5
  ///  - min_range = 0
  ///  - max_range = 1
  ///
  static const DepthSensorSpecification& get_octant_1_spec() {
    static const DepthSensorSpecification spec(0,         // min_theta
                                               M_PI / 2,  // max_theta
                                               0,         // min_phi
                                               M_PI / 2,  // max_phi
                                               10,        // num_theta_values
                                               5,         // num_phi_values
                                               0,         // min_range
                                               1);        // max_range
    return spec;
  }

  /// Returns a DepthSensorSpecification that covers the sensor's base frame's
  /// X/Y plane. It contains the following specifications:
  ///
  ///  - min_theta = -M_PI
  ///  - max_theta = M_PI
  ///  - min_phi = 0
  ///  - max_phi = 0
  ///  - num_theta_values = 50
  ///  - num_phi_values = 1
  ///  - min_range = 0
  ///  - max_range = 1
  ///
  static const DepthSensorSpecification& get_xy_planar_spec() {
    static const DepthSensorSpecification spec(-M_PI,  // min_theta
                                               M_PI,   // max_theta
                                               0,      // min_phi
                                               0,      // max_phi
                                               50,     // num_theta_values
                                               1,      // num_phi_values
                                               0,      // min_range
                                               1);     // max_range
    return spec;
  }

  /// Returns a DepthSensorSpecification that covers the sensor's base frame's
  /// X/Z plane. It contains the following specifications:
  ///
  ///  - min_theta = 0
  ///  - max_theta = 0
  ///  - min_phi = -M_PI / 2
  ///  - max_phi = M_PI / 2
  ///  - num_theta_values = 1
  ///  - num_phi_values = 50
  ///  - min_range = 0
  ///  - max_range = 1
  ///
  static const DepthSensorSpecification& get_xz_planar_spec() {
    static const DepthSensorSpecification spec(0,          // min_theta
                                               0,          // max_theta
                                               -M_PI / 2,  // min_phi
                                               M_PI / 2,   // max_phi
                                               1,          // num_theta_values
                                               50,         // num_phi_values
                                               0,          // min_range
                                               1);         // max_range
    return spec;
  }

  /// Returns a DepthSensorSpecification that covers all 8 octants of the
  /// sensor's base frame. The sensed region is in the shape of a hollow
  /// sphere with a radius between min_range and max_range. It contains the
  /// following specifications:
  ///
  ///  - min_theta = -M_PI
  ///  - max_theta = M_PI
  ///  - min_phi = -M_PI / 2
  ///  - max_phi = M_PI / 2
  ///  - num_theta_values = 50
  ///  - num_phi_values = 50
  ///  - min_range = 0
  ///  - max_range = 1
  ///
  /// Note that since min_range is 0, the sensed sphere is actually solid.
  static const DepthSensorSpecification& get_xyz_spherical_spec() {
    static const DepthSensorSpecification spec(-M_PI,      // min_theta
                                               M_PI,       // max_theta
                                               -M_PI / 2,  // min_phi
                                               M_PI / 2,   // max_phi
                                               50,         // num_theta_values
                                               50,         // num_phi_values
                                               0,          // min_range
                                               1);         // max_range
    return spec;
  }

  /// Returns a DepthSensorSpecification that consists of a single depth
  /// measurement along the sensor's base frame's +X axis between 1 meter and 2
  /// meters. It contains the following specifications:
  ///
  ///  - min_theta = 0
  ///  - max_theta = 0
  ///  - min_phi = 0
  ///  - max_phi = 0
  ///  - num_theta_values = 1
  ///  - num_phi_values = 1
  ///  - min_range = 1
  ///  - max_range = 2
  ///
  static const DepthSensorSpecification& get_x_linear_spec() {
    static const DepthSensorSpecification spec(0,   // min_theta
                                               0,   // max_theta
                                               0,   // min_phi
                                               0,   // max_phi
                                               1,   // num_theta_values
                                               1,   // num_phi_values
                                               1,   // min_range
                                               2);  // max_range
    return spec;
  }
  ///@}
 private:
  void update_num_depth_readings();
  void update_theta_increment();
  void update_phi_increment();

  double min_theta_{};
  double max_theta_{};
  double min_phi_{};
  double max_phi_{};
  int num_theta_values_{};
  int num_phi_values_{};
  int num_depth_readings_{};
  double min_range_{0};
  double max_range_{std::numeric_limits<double>::max()};

  // The following variables are computed based on the above variables.
  double theta_increment_{};
  double phi_increment_{};
};

inline bool operator==(const DepthSensorSpecification& lhs,
                       const DepthSensorSpecification& rhs) {
  return lhs.min_theta() == rhs.min_theta() &&
         lhs.max_theta() == rhs.max_theta() && lhs.min_phi() == rhs.min_phi() &&
         lhs.max_phi() == rhs.max_phi() &&
         lhs.num_theta_values() == rhs.num_theta_values() &&
         lhs.num_phi_values() == rhs.num_phi_values() &&
         lhs.num_depth_readings() == rhs.num_depth_readings() &&
         lhs.min_range() == rhs.min_range() &&
         lhs.max_range() == rhs.max_range() &&
         lhs.theta_increment() == rhs.theta_increment() &&
         lhs.phi_increment() == rhs.phi_increment();
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
