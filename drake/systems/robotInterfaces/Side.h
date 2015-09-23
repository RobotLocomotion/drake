#ifndef SYSTEMS_CONTROLLERS_SIDE_H_
#define SYSTEMS_CONTROLLERS_SIDE_H_

#include <array>
#include <string>

#undef DLLEXPORT_DRAKESIDE
#if defined(WIN32) || defined(WIN64)
  #if defined(drakeSide_EXPORTS)
    #define DLLEXPORT_DRAKESIDE __declspec( dllexport )
  #else
    #define DLLEXPORT_DRAKESIDE __declspec( dllimport )
  #endif
#else
  #define DLLEXPORT_DRAKESIDE
#endif


// adapted from https://en.wikibooks.org/wiki/More_C%2B%2B_Idioms/Type_Safe_Enum
class DLLEXPORT_DRAKESIDE Side
{
public:
  enum SideEnum {LEFT, RIGHT};

private:
  SideEnum val;

public:
  static const std::array<SideEnum, 2> values;
  Side(); // to allow usage in STL containers
  Side(SideEnum v);
  SideEnum underlying() const { return val; }

  bool operator == (const Side & other) const;
  bool operator != (const Side & other) const;
  bool operator <  (const Side & other) const;
  bool operator <= (const Side & other) const;
  bool operator >  (const Side & other) const;
  bool operator >= (const Side & other) const;

  Side oppositeSide()  const;
  std::string toString() const;
};

#endif /* SYSTEMS_CONTROLLERS_SIDE_H_ */
