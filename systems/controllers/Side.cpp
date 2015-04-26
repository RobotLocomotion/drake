#include "Side.h"
#include <stdexcept>

const std::array<Side::SideEnum, 2> Side::values = { { Side::LEFT, Side::RIGHT } };

Side::Side() :
    val(LEFT)
{
  // empty
}

Side::Side(Side::SideEnum v) :
    val(v)
{
  // empty
}

bool Side::operator==(const Side & other) const
{
  return val == other.val;
}

bool Side::operator!=(const Side & other) const
{
  return val != other.val;
}

bool Side::operator<(const Side & other) const
{
  return val < other.val;
}

bool Side::operator<=(const Side & other) const
{
  return val <= other.val;
}

bool Side::operator>(const Side & other) const
{
  return val > other.val;
}

bool Side::operator>=(const Side & other) const
{
  return val >= other.val;
}

Side Side::oppositeSide() const
{
  switch (val) {
  case LEFT:
    return Side(RIGHT);
  case RIGHT:
    return Side(LEFT);
  default:
    throw std::runtime_error("should not get here");
  }
}

std::string Side::toString() const
{
  switch (val) {
  case LEFT:
    return "left";
  case RIGHT:
    return "right";
  default:
    throw std::runtime_error("should not get here");
  }
}
