#pragma once

#include <map>

#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/type_specific_identifier.h"
#include "drake/common/drake_copyable.h"

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
std::map<BulbColor, const char*> BulbColorMapper();

/// Defines the possible bulb types.
enum class BulbType {
  kRound = 0,
  kArrow,
};

/// Maps BulbType enums to string representations.
std::map<BulbType, const char*> BulbTypeMapper();

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
  /// frame should approximate the bulb group's CoM.
  ///
  /// @param orientation_bulb_group The rotational offset of this bulb's frame
  /// relative to the frame of the bulb group that contains it. The +Z axis
  /// should align with the bulb's "up" direction, and the +X axis should point
  /// in the direction that the bulb is facing. If the bulb type is an arrow,
  /// the +Y axis should point in the same direction as the arrow. Otherwise,
  /// there is no restriction on the +Y axis.
  ///
  /// @param color The color of this bulb.
  ///
  /// @param type The type of this bulb.
  Bulb(const Id& id, const GeoPosition& position_bulb_group,
       const Rotation& orientation_bulb_group, const BulbColor& color,
       const BulbType& type);

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

  /// Returns the Bulb instances contained within this Bulb.
  const BulbType& type() const { return type_; }

 private:
  Id id_;
  GeoPosition position_bulb_group_;
  Rotation orientation_bulb_group_;
  BulbColor color_ = BulbColor::kRed;
  BulbType type_ = BulbType::kRound;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
