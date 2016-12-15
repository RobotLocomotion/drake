#pragma once

#include <cmath>
#include <limits>

namespace drake {
namespace multibody {

/// Holds the specifications of an IdealDepthSensor.
///
/// @see IdealDepthSensor.
class IdealDepthSensorSpecification {
 public:
  IdealDepthSensorSpecification(double min_theta, double max_theta,
                                double min_phi, double max_phi,
                                int num_theta_values, int num_phi_values,
                                double min_range, double max_range);

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

  /// @name Convenient instantiations of the IdealDepthSensorSpecification.
  /// The following are frequently used IdealDepthSensorSpecification that can
  /// either be
  /// used directly or as templates for specifying more specific specifications.
  ///@{
  static const IdealDepthSensorSpecification& get_octant_1_spec() {
    static const IdealDepthSensorSpecification spec(0,         // min_theta
                                                    M_PI / 2,  // max_theta
                                                    0,         // min_phi
                                                    M_PI / 2,  // max_phi
                                                    10,  // num_theta_values
                                                    5,   // num_phi_values
                                                    0,   // min_range
                                                    1);  // max_range
    return spec;
  }

  static const IdealDepthSensorSpecification& get_xy_planar_spec() {
    static const IdealDepthSensorSpecification spec(-M_PI,  // min_theta
                                                    M_PI,   // max_theta
                                                    0,      // min_phi
                                                    0,      // max_phi
                                                    50,     // num_theta_values
                                                    1,      // num_phi_values
                                                    0,      // min_range
                                                    1);     // max_range
    return spec;
  }

  static const IdealDepthSensorSpecification& get_xz_planar_spec() {
    static const IdealDepthSensorSpecification spec(0,          // min_theta
                                                    0,          // max_theta
                                                    -M_PI / 2,  // min_phi
                                                    M_PI / 2,   // max_phi
                                                    1,   // num_theta_values
                                                    50,  // num_phi_values
                                                    0,   // min_range
                                                    1);  // max_range
    return spec;
  }

  static const IdealDepthSensorSpecification& get_xyz_spherical_spec() {
    static const IdealDepthSensorSpecification spec(-M_PI,      // min_theta
                                                    M_PI,       // max_theta
                                                    -M_PI / 2,  // min_phi
                                                    M_PI / 2,   // max_phi
                                                    50,  // num_theta_values
                                                    50,  // num_phi_values
                                                    0,   // min_range
                                                    1);  // max_range
    return spec;
  }

  static const IdealDepthSensorSpecification& get_x_linear_spec() {
    static const IdealDepthSensorSpecification spec(0,   // min_theta
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

inline bool operator==(const IdealDepthSensorSpecification& lhs,
                       const IdealDepthSensorSpecification& rhs) {
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

}  // namespace multibody
}  // namespace drake
