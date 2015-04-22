#ifndef SYSTEMS_CONTROLLERS_SIDE_H_
#define SYSTEMS_CONTROLLERS_SIDE_H_

#include <vector>
#include <string>

// adapted from https://en.wikibooks.org/wiki/More_C%2B%2B_Idioms/Type_Safe_Enum
class Side
{
public:
  enum SideEnum {LEFT, RIGHT};

private:
  SideEnum val;

public:
  Side(SideEnum v) : val(v) {}
  SideEnum underlying() const { return val; }

  bool operator == (const Side & other);
  bool operator != (const Side & other);
  bool operator <  (const Side & other);
  bool operator <= (const Side & other);
  bool operator >  (const Side & other);
  bool operator >= (const Side & other);

  Side oppositeSide();
  std::string toString();
  static std::vector<Side> values();
};

#endif /* SYSTEMS_CONTROLLERS_SIDE_H_ */
