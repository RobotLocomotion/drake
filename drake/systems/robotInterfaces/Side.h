#pragma once

#include <array>
#include <string>
#include "drake/common/drake_export.h"

// adapted from https://en.wikibooks.org/wiki/More_C%2B%2B_Idioms/Type_Safe_Enum
// TODO(tkoolen): replace with enum class
class DRAKE_EXPORT Side {
 public:
  enum SideEnum { LEFT, RIGHT };

 private:
  SideEnum val;

 public:
  static const std::array<SideEnum, 2> values;

  Side();  // to allow usage in STL containers

  // NOLINTNEXTLINE(runtime/explicit) This conversion is desirable.
  Side(SideEnum v);

  SideEnum underlying() const { return val; }

  bool operator==(const Side& other) const;
  bool operator!=(const Side& other) const;
  bool operator<(const Side& other) const;
  bool operator<=(const Side& other) const;
  bool operator>(const Side& other) const;
  bool operator>=(const Side& other) const;

  Side oppositeSide() const;
  std::string toString() const;
};
