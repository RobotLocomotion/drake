#ifndef __CoordinateFrame_H__
#define __CoordinateFrame_H__

#include <string>
#include <vector>

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

private:
  std::string name;
  unsigned int dim;
  std::string prefix;
  std::vector<std::string> coordinates;
};

#endif // #define __CoordinateFrame_H_