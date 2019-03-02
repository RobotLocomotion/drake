#pragma once

#include <unordered_map>
#include <vector>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/type_specific_identifier.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/hash.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

/// Defines the possible bulb colors.
enum class BulbColor {
  kRed = 0,
  kYellow,
  kGreen,
};

/// Maps BulbColor enums to string representations.
std::unordered_map<BulbColor, const char*, DefaultHash> BulbColorMapper();

/// Defines the possible bulb types.
enum class BulbType {
  kRound = 0,
  kArrow,
};

/// Maps BulbType enums to string representations.
std::unordered_map<BulbType, const char*, DefaultHash> BulbTypeMapper();

/// Models a bulb within a bulb group.
class Bulb final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Bulb);

  /// Unique identifier for a Bulb.
  using Id = TypeSpecificIdentifier<class Bulb>;

  /// Constructs a Bulb instance.
  ///
  /// @param id The bulb's unique ID.
  ///
  /// @param position_bulb_group The linear offset of this bulb's frame relative
  /// to the frame of the bulb group that contains it. The origin of this bulb's
  /// frame should approximate the bulb's CoM.
  ///
  /// @param orientation_bulb_group The rotational offset of this bulb's frame
  /// relative to the frame of the bulb group that contains it. The +Z axis
  /// should align with the bulb's "up" direction and the +X axis should point
  /// in the direction that the bulb is facing. Following a right-handed
  /// coordinate frame, the +Y axis should point left when facing the +X
  /// direction.
  ///
  /// @param color The color of this bulb.
  ///
  /// @param type The type of this bulb.
  ///
  /// @param arrow_orientation_rad The orientation of the arrow when @p type is
  /// BulbType::kArrow. This is the angle along the bulb's +X axis relative to
  /// the bulb's +Y axis. For example, an angle of zero means the bulb's arrow
  /// is pointing along the bulb's +Y axis, which means it's a right-turn arrow
  /// when viewed by an approaching vehicle. Similarly, an angle of PI/2 points
  /// in the bulb's +Z direction (i.e., forward from an approaching vehicle's
  /// perspective), and an angle of PI points to in the bulb's -Y direction
  /// (i.e., left from an approaching vehicle's perspective). This parameter
  /// must be defined when @p type is BulbType::kArrow, otherwise an exception
  /// will be thrown. An exception will also be thrown if this parameter is
  /// defined for non-arrow BulbType values.
  Bulb(const Id& id, const GeoPosition& position_bulb_group,
       const Rotation& orientation_bulb_group, const BulbColor& color,
       const BulbType& type,
       const optional<double>& arrow_orientation_rad = nullopt);

  /// Returns this Bulb instance's unique identifier.
  const Id& id() const { return id_; }

  /// Returns the linear offset of this bulb's frame relative to the frame of
  /// the bulb group that contains it.
  const GeoPosition& position_bulb_group() const {
    return position_bulb_group_;
  }

  /// Returns the rotational offset of this bulb's frame relative to the frame
  /// of the bulb group that contains it.
  const Rotation& orientation_bulb_group() const {
    return orientation_bulb_group_;
  }

  /// Returns the color of this bulb.
  const BulbColor& color() const { return color_; }

  /// Returns the type of this bulb.
  const BulbType& type() const { return type_; }

  /// Returns the arrow's orientation. Only applicable if type() returns
  /// BulbType::kArrow. See constructor's documentation for semantics.
  const optional<double> arrow_orientation_rad() const {
    return arrow_orientation_rad_;
  }

 private:
  Id id_;
  GeoPosition position_bulb_group_;
  Rotation orientation_bulb_group_;
  BulbColor color_ = BulbColor::kRed;
  BulbType type_ = BulbType::kRound;
  optional<double> arrow_orientation_rad_ = nullopt;
};

/// Models a group of bulbs within a traffic light. All of the bulbs within a
/// group should share the same approximate orientation. However, this is not
/// programmatically enforced.
class BulbGroup final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BulbGroup);

  /// Unique identifier for a BulbGroup.
  using Id = TypeSpecificIdentifier<BulbGroup>;

  /// Constructs a BulbGroup instance.
  ///
  /// @param id The bulb group's unique ID.
  ///
  /// @param position_traffic_light The linear offset of this bulb group's frame
  /// relative to the frame of the traffic light that contains it. The origin of
  /// this bulb group's frame should approximate the bulb group's CoM.
  ///
  /// @param orientation_traffic_light The rotational offset of this bulb
  /// group's frame relative to the frame of the traffic light that contains it.
  /// The +Z axis should align with the bulb group's "up" direction, and the +X
  /// axis should point in the direction that the bulb group is facing.
  /// Following a right-handed coordinate frame, the +Y axis should point left
  /// when facing the +X direction.
  ///
  /// @param bulbs The bulbs that are part of this BulbGroup.
  BulbGroup(const Id& id, const GeoPosition& position_traffic_light,
            const Rotation& orientation_traffic_light,
            const std::vector<Bulb>& bulbs);

  /// Returns this BulbGroup instance's unique identifier.
  const Id& id() const { return id_; }

  /// Returns the linear offset of this bulb group's frame relative to the
  /// frame of the traffic light that contains it.
  const GeoPosition& position_traffic_light() const {
    return position_traffic_light_;
  }

  /// Returns the rotational offset of this bulb group's frame relative to the
  /// frame of the traffic light that contains it.
  const Rotation& orientation_traffic_light() const {
    return orientation_traffic_light_;
  }

  /// Returns the bulbs contained within this bulb group.
  const std::vector<Bulb>& bulbs() const { return bulbs_; }

 private:
  Id id_;
  GeoPosition position_traffic_light_;
  Rotation orientation_traffic_light_;
  std::vector<Bulb> bulbs_;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
