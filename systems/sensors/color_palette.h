#pragma once

#include <functional>
#include <ostream>
#include <unordered_map>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/hash.h"

namespace drake {
namespace systems {
namespace sensors {

/// Holds r, g, b values to represent a color pixel.
///
/// @tparam T A type for each color channel.
template <typename T>
struct Color {
  T r;  /// Red.
  T g;  /// Green.
  T b;  /// Blue.

  bool operator==(const Color<T>& other) const {
    return this->r == other.r && this->g == other.g && this->b == other.b;
  }

  /// Implements the @ref hash_append concept.
  template <class HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher, const Color& item) noexcept {
    using drake::hash_append;
    hash_append(hasher, item.r);
    hash_append(hasher, item.g);
    hash_append(hasher, item.b);
  }
};

template <typename T>
std::ostream& operator<<(std::ostream& out, const Color<T>& color) {
  out << "(" << color.r << ", " << color.g << ", " << color.b << ")";
  return out;
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake

namespace std {
template <typename T>
struct hash<drake::systems::sensors::Color<T>>
    : public drake::DefaultHash {};
}  // namespace std

namespace drake {
namespace systems {
namespace sensors {

/// Defines a color based on its three primary additive colors: red, green, and
/// blue. Each of these primary additive colors are in the range of [0, 255].
using ColorI = Color<int>;

/// Defines a color based on its three primary additive colors: red, green, and
/// blue. Each of these primary additive colors are in the range of [0, 1].
using ColorD = Color<double>;

// TODO(SeanCurtis-TRI): As indicated in #9628, provide unit tests for the
// contents of this file.
/// Creates and holds a palette of colors for visualizing different objects in a
/// scene (the intent is for a different color to be applied to each identified
/// object). The colors are chosen so as to be easily distinguishable. In other
/// words, the intensities are spaced as widely as possible given the number of
/// required colors. Black, white and gray, which has the same value for all the
/// three color channels, are not part of this color palette. This color palette
/// can hold up to 1535 colors.
///
/// @tparam IdType  The type of value used for label values.
template <typename IdType>
class ColorPalette {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ColorPalette)

  /// A constructor for %ColorPalette.
  ///
  /// @param num_colors The number of colors that you want ColorPalette to hold.
  /// We assume this will be the number of rigid bodies in rendering scene.
  ///
  /// @param terrain_id The id to express pixels which correspond to flat
  /// terrain. This will be used in the label image.
  ///
  /// @param no_body_id The id to express pixels that have no body. This will be
  ///  used in the label image.
  ///
  /// @throws std::logic_error When @p num_colors exceeds the maximum limit,
  /// which is 1535.
  ColorPalette(int num_colors, IdType terrain_id, IdType no_body_id)
      : terrain_id_(terrain_id), empty_id_(no_body_id) {
    // Dividing by six because we create "bands" of colors at various intensity
    // levels and the band width is six. In other words, six is the number of
    // `push_back` calls in the `for` loop below.
    const int num = std::ceil(num_colors / 6.);
    if (num >= 256) {  // Larger than the maximum number of uint8_t.
      throw std::logic_error("num_colors exceeded the maximum number.");
    }

    for (int i = 0; i < num; ++i) {
      // It is possible to have more colors, but we want the colors to be as
      // distinguishable as possible for visualization purpose.  We can add more
      // colors as needed.
      const int intensity = 255 - i * 255 / num;
      DRAKE_ASSERT(intensity > 0);
      colors_.push_back(ColorI{intensity, 0, 0});
      colors_.push_back(ColorI{0, intensity, 0});
      colors_.push_back(ColorI{0, 0, intensity});
      colors_.push_back(ColorI{intensity, intensity, 0});
      colors_.push_back(ColorI{0, intensity, intensity});
      colors_.push_back(ColorI{intensity, 0, intensity});
    }

    // Creates hash map for ID look up.
    for (size_t i = 0; i < colors_.size(); ++i) {
      color_id_map_[colors_[i]] = IdType(i);
    }
    color_id_map_[kTerrainColor] = terrain_id;
    color_id_map_[kSkyColor] = no_body_id;
  }

  /// Takes ColorI whose pixel range is [0, 255] and returns ColorD whose pixel
  /// range is [0, 1].
  ///
  /// @param color An input color to be normalized.
  static ColorD Normalize(const ColorI& color) {
    ColorD normalized;
    normalized.r = color.r / 255.;
    normalized.g = color.g / 255.;
    normalized.b = color.b / 255.;
    return normalized;
  }

  /// Returns a color of type ColorI which corresponds to given index.
  /// The pixel range of returned color is [0, 255].
  ///
  /// @param index An index that corresponds to the color to be returned.
  const ColorI& get_color(IdType index) const {
    if (index == terrain_id_) {
      return get_terrain_color();
    } else if (index == empty_id_) {
      return get_sky_color();
    } else {
      DRAKE_DEMAND(0 <= index && index < static_cast<int>(colors_.size()));
      return colors_[index];
    }
  }

  /// Returns a color of type ColorD which corresponds to given index.
  /// The pixel range of returned color is [0, 1].
  ///
  /// @param index An index that corresponds to the color to be returned.
  ColorD get_normalized_color(IdType index) const {
    ColorD color = Normalize(get_color(index));
    return color;
  }

  /// Returns the color of type ColorI which corresponds to sky.
  /// The pixel range of returned color is [0, 255].
  const ColorI& get_sky_color() const {
    return kSkyColor;
  }

  /// Returns the color of type ColorI which corresponds to flat terrain.
  /// The pixel range of returned color is [0, 255].
  const ColorI& get_terrain_color() const {
    return kTerrainColor;
  }

  /// Looks up the ID which corresponds to the given color.
  ///
  /// @param color The color you want to know the corresponding ID.
  IdType LookUpId(const ColorI& color) const {
    return color_id_map_.at(color);
  }

 private:
  // These colors are chosen so as to be easily distinguished from the colors in
  // colors_. They are guaranteed to be distinct from colors_ because none of
  // their intensity elements are identical.
  // TODO(kunimatsu-tri) Add support for arbitrary colors for the terrain and
  // the sky.
  const ColorI kTerrainColor{255, 229, 204};
  const ColorI kSkyColor{204, 229, 255};
  std::vector<ColorI> colors_;
  std::unordered_map<ColorI, IdType> color_id_map_;
  IdType terrain_id_{};
  IdType empty_id_{};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
