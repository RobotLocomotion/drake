#include "Side.h"
#include <stdexcept>

bool Side::operator==(const Side & other)
{
  return val == other.val;
}

bool Side::operator!=(const Side & other)
{
  return val != other.val;
}

bool Side::operator<(const Side & other)
{
  return val < other.val;
}

bool Side::operator<=(const Side & other)
{
  return val <= other.val;
}

bool Side::operator>(const Side & other)
{
  return val > other.val;
}

bool Side::operator>=(const Side & other)
{
  return val >= other.val;
}

Side Side::oppositeSide()
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

std::string Side::toString()
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

std::vector<Side> Side::values()
{
  return std::vector<Side> { { Side::LEFT, Side::RIGHT } };
}
