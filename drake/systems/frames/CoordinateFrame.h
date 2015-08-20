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

/// Every input, state, and output in a DynamicalSystem has a coordinate frame
/// attached to it.  Many bugs can be avoided by forcing developers to be
/// explicit about these coordinate systems when they make combinations of systems.

class DLLEXPORT_FRAME CoordinateFrame {
public:
  CoordinateFrame(std::string& _name, unsigned int _dim, std::vector<std::string>& _coordinates)
          : name(_name), dim(_dim), coordinates(_coordinates) {};
  CoordinateFrame(std::string& _name, unsigned int _dim)
          : name(_name), dim(_dim) {
    for (int i=0; i<dim; i++) coordinates.push_back("x"+std::to_string(i)); // todo: update this if I bring the prefix logic over
  }
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
  std::string name;  // a descriptive name for this coordinate frame
  unsigned int dim;  // number of elements in the coordinate vector
  std::vector<std::string> coordinates; // a string name for each element in the vector (size==dim)
};

#endif // #define __CoordinateFrame_H_