#ifndef __CoordinateFrame_H__
#define __CoordinateFrame_H__

#include <string>
#include <vector>
#include <exception>

#undef DLLEXPORT_FRAME
#if defined(WIN32) || defined(WIN64)
#if defined(drakeCoordinateFrame_EXPORTS)
    #define DLLEXPORT_FRAME __declspec( dllexport )
  #else
    #define DLLEXPORT_FRAME __declspec( dllimport )
  #endif
#else
#define DLLEXPORT_FRAME
#endif

class DLLEXPORT_FRAME CoordinateFrame {
public:
  CoordinateFrame(std::string& _name, unsigned int _dim, std::string& _prefix, std::vector<std::string>& _coordinates)
          : name(_name), dim(_dim), prefix(_prefix), coordinates(_coordinates) {};
  virtual ~CoordinateFrame(void) {};

  const std::string& getName() const { return name; };
  const unsigned int getDim() const { return dim; };
  const std::string& getCoordinateName(unsigned int i) const {
    if (i>=coordinates.size())
      throw std::runtime_error("index exceeds dimension of coordinate frame");
    return coordinates[i];
  }
  const std::vector<std::string>& getCoordinateNames() const { return coordinates; }
  void setCoordinateNames(const std::vector<std::string>& _coordinates)
  {
    if (_coordinates.size()!=dim)
      throw std::runtime_error("coordinates must be an array of dim strings");
    coordinates = _coordinates;
  }

private:
  std::string name;
  unsigned int dim;
  std::string prefix;
  std::vector<std::string> coordinates;
};

#endif // #define __CoordinateFrame_H_