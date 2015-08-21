#ifndef __CoordinateFrame_H__
#define __CoordinateFrame_H__

#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <exception>
#include <initializer_list>

#undef DLLEXPORT
#if defined(WIN32) || defined(WIN64)
#if defined(drakeCoordinateFrame_EXPORTS)
#define DLLEXPORT __declspec( dllexport )
#else
#define DLLEXPORT __declspec( dllimport )
#endif
#else
#define DLLEXPORT
#endif


/// Every input, state, and output in a DynamicalSystem has a coordinate frame
/// attached to it.  Many bugs can be avoided by forcing developers to be
/// explicit about these coordinate systems when they make combinations of systems.

class DLLEXPORT CoordinateFrame {
public:
  CoordinateFrame(const std::string& _name, const std::vector<std::string>& _coordinates)
          : name(_name), coordinates(_coordinates) {};
  CoordinateFrame(const std::string& _name, const unsigned int dim, const std::string& prefix)
          : name(_name) {
    for (int i=0; i<dim; i++) coordinates.push_back(prefix+std::to_string(i));
  }
  CoordinateFrame(const std::string& _name) : name(_name) {}

  virtual ~CoordinateFrame(void) {};

  const std::string& getName() const { return name; };
  const unsigned int getDim() const { return coordinates.size(); };
  const std::string& getCoordinateName(unsigned int i) const {
    if (i>=coordinates.size())
      throw std::runtime_error("index exceeds dimension of coordinate frame");
    return coordinates[i];
  }
  const std::vector<std::string>& getCoordinateNames() const { return coordinates; }
  void setCoordinateNames(const std::vector<std::string>& _coordinates)
  {
    coordinates = _coordinates;
  }

  virtual std::ostream& print(std::ostream& os) const {
    os << "Coordinate Frame: " << name << " (" << getDim() << " elements)" << std::endl;

    for (auto c : coordinates) {
      os << "  " << c << std::endl;
    }
    return os;
  }

  friend std::ostream& operator<<(std::ostream& os, const CoordinateFrame& m)
  {
    return m.print(os);
  }

protected:
  std::string name;  // a descriptive name for this coordinate frame
  std::vector<std::string> coordinates; // a string name for each element in the vector (size==dim)
};


class DLLEXPORT MultiCoordinateFrame : public CoordinateFrame {
public:
  MultiCoordinateFrame(const std::string& name, std::initializer_list<std::shared_ptr<CoordinateFrame>> _frames);
  virtual ~MultiCoordinateFrame(void) {};

  virtual std::ostream& print(std::ostream& os) const {
    os << "Multi-Coordinate Frame: " << name << " (" << getDim() << " elements)" << std::endl;

    for (auto sf : frames) {
      os << "  " << sf.frame->getName() << " (" << sf.frame->getDim() << " elements) " << std::endl;
    }
    return os;
  }

  static std::shared_ptr<CoordinateFrame> constructFrame(const std::string& name, std::initializer_list<std::shared_ptr<CoordinateFrame>> _frames) {
    unsigned int count=0;
    for (auto f : _frames)
      if (f) count++;
    if (count<1) return nullptr;
    else if (count==1) {
      for (auto f : _frames)
        if (f) return f;
      return nullptr;  // shouldn't get here, but makes the compiler happier
    } else {
      return std::shared_ptr<CoordinateFrame>(new MultiCoordinateFrame(name,_frames));
    }
  }


private:
  /// Store the data in two (compatible) ways:
  ///   1) as a list of subframes, with a map to the relevant indices in the multi-frame
  ///   2) as a list of coordinates, with references to the associated subframe
  struct SubFrame {
    std::shared_ptr<CoordinateFrame> frame;
    std::vector<unsigned int> coordinate_indices; // which indices in the multi-frame are associated with this sub-frame
  };
  struct CoordinateRef {
    std::shared_ptr<CoordinateFrame> frame;
    unsigned int index_in_subframe;
  };
  std::vector<struct SubFrame> frames;
  std::vector<struct CoordinateRef> coordinate_refs;
};


#endif // #define __CoordinateFrame_H_