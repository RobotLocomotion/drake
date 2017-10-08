#pragma once

#include <cmath>
#include <limits>
#include <string>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace systems {
namespace sensors {

/// Holds a DepthSensor's specification.
///
/// @see DepthSensor.
class DepthSensorSpecification {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DepthSensorSpecification)

  /// Constructs a %DepthsensorSpecification with all default values.
  DepthSensorSpecification() {}

  /// Constructs a fully-defined %DepthSensorSpecification.
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
  DepthSensorSpecification(double min_yaw, double max_yaw,
                           double min_pitch, double max_pitch,
                           int num_yaw_values, int num_pitch_values,
                           double min_range, double max_range);

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
  double min_range() const { return min_range_; }
  double max_range() const { return max_range_; }
  ///@}

  /// @name Accessors to automatically derived parameters.
  ///
  /// The following methods return values that are derived based on the manually
  /// specified parameters.
  ///@{
  int num_depth_readings() const;
  double yaw_increment() const;
  double pitch_increment() const;
  ///@}

  void set_min_yaw(double min_yaw);
  void set_max_yaw(double max_yaw);
  void set_min_pitch(double min_pitch);
  void set_max_pitch(double max_pitch);
  void set_num_yaw_values(int num_yaw_values);
  void set_num_pitch_values(int num_pitch_values);
  void set_min_range(double min_range);
  void set_max_range(double max_range);

  /// @name Methods for the convenient initialization of commonly-used
  /// DepthSensorSpecification specifications.
  ///
  /// The following methods provide frequently used DepthSensorSpecification
  /// specifications that can be used directly or as templates for
  /// fine-tuning.
  ///@{

  /// Sets @p spec to specify a sensor that  covers octant 1 of the sensor's
  /// base frame. It contains the following specifications:
  ///
  ///  - min_yaw = 0
  ///  - max_yaw = M_PI_2
  ///  - min_pitch = 0
  ///  - max_pitch = M_PI_2
  ///  - num_yaw_values = 10
  ///  - num_pitch_values = 5
  ///  - min_range = 0
  ///  - max_range = 1
  ///
  static void set_octant_1_spec(DepthSensorSpecification* spec) {
    spec->set_min_yaw(0);
    spec->set_max_yaw(M_PI_2);
    spec->set_min_pitch(0);
    spec->set_max_pitch(M_PI_2);
    spec->set_num_yaw_values(10);
    spec->set_num_pitch_values(5);
    spec->set_min_range(0);
    spec->set_max_range(1);
  }

  /// Sets @p spec to specify a sensor that covers the sensor's base frame's
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
  /// The specification describes a "planar" depth sensor in the sense that
  /// `min_pitch` and `max_pitch` are both zero whereas the yaw range is
  /// non-zero. Thus, it scans in the yaw direction, which is in the plane
  /// formed by the X and Y axes of the sensor's base frame. Example depth
  /// sensors that fit this description include those made by Hokuyo and SICK.
  static void set_xy_planar_spec(DepthSensorSpecification* spec) {
    spec->set_min_yaw(-M_PI);
    spec->set_max_yaw(M_PI);
    spec->set_min_pitch(0);
    spec->set_max_pitch(0);
    spec->set_num_yaw_values(50);
    spec->set_num_pitch_values(1);
    spec->set_min_range(0);
    spec->set_max_range(1);
  }

  /// Sets @p spec to specify a sensor that covers the sensor's base frame's
  /// X/Z plane. It contains the following specifications:
  ///
  ///  - min_yaw = 0
  ///  - max_yaw = 0
  ///  - min_pitch = -M_PI_2
  ///  - max_pitch = M_PI_2
  ///  - num_yaw_values = 1
  ///  - num_pitch_values = 50
  ///  - min_range = 0
  ///  - max_range = 1
  ///
  /// See the description of get_xy_planar_spec() for an explanation on why the
  /// returned specification describes a "planar" depth sensor. In this case,
  /// `min_yaw` and `max_yaw` are both equal to zero whereas the pitch range is
  /// not zero, meaning the described sensor scans its X / Z plane.
  static void set_xz_planar_spec(DepthSensorSpecification* spec) {
    spec->set_min_yaw(0);
    spec->set_max_yaw(0);
    spec->set_min_pitch(-M_PI_2);
    spec->set_max_pitch(M_PI_2);
    spec->set_num_yaw_values(1);
    spec->set_num_pitch_values(50);
    spec->set_min_range(0);
    spec->set_max_range(1);
  }

  /// Sets @p spec to specify a sensor that covers all 8 octants of the sensor's
  /// base frame. The sensed region is in the shape of a sphere with a radius
  /// between min_range and max_range. It contains the following specifications:
  ///
  ///  - min_yaw = -M_PI
  ///  - max_yaw = M_PI
  ///  - min_pitch = -M_PI_2
  ///  - max_pitch = M_PI_2
  ///  - num_yaw_values = 50
  ///  - num_pitch_values = 50
  ///  - min_range = 0
  ///  - max_range = 1
  ///
  /// Note that since min_range is 0, the sensed sphere is solid.
  ///
  static void set_xyz_spherical_spec(DepthSensorSpecification* spec) {
    spec->set_min_yaw(-M_PI);
    spec->set_max_yaw(M_PI);
    spec->set_min_pitch(-M_PI_2);
    spec->set_max_pitch(M_PI_2);
    spec->set_num_yaw_values(50);
    spec->set_num_pitch_values(50);
    spec->set_min_range(0);
    spec->set_max_range(1);
  }

  /// Sets @p spec to specify a sensor with a single depth measurement along the
  /// sensor's base frame's +X axis between 1 meter and 2 meters. It contains
  /// the following specifications:
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
  static void set_x_linear_spec(DepthSensorSpecification* spec) {
    spec->set_min_yaw(0);
    spec->set_max_yaw(0);
    spec->set_min_pitch(0);
    spec->set_max_pitch(0);
    spec->set_num_yaw_values(1);
    spec->set_num_pitch_values(1);
    spec->set_min_range(1);
    spec->set_max_range(2);
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
  int num_yaw_values_{1};
  int num_pitch_values_{1};
  double min_range_{0};
  double max_range_{std::numeric_limits<double>::max()};
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
