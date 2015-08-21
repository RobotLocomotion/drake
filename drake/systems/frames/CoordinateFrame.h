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

class DrakeSystem;
typedef std::shared_ptr<DrakeSystem> DrakeSystemPtr;

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

  virtual const unsigned int getDim() const { return coordinates.size(); };
  virtual const std::string& getCoordinateName(unsigned int i) const {
    if (i>=coordinates.size())
      throw std::runtime_error("index exceeds dimension of coordinate frame");
    return coordinates[i];
  }
  virtual std::vector<std::string> getCoordinateNames() const { return coordinates; }
  virtual void setCoordinateNames(const std::vector<std::string>& _coordinates)
  {
    if (_coordinates.size()!=coordinates.size())
      throw std::runtime_error("(CoordinateFrame:setCoordinateNames) this would change the dimension of the frame");  // which would, e.g. confuse a multi-coordinate frame that was pointing to this
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

  virtual DrakeSystemPtr setupLCMOutputs(DrakeSystemPtr sys) { return sys; }

  std::string name;  // a descriptive name for this coordinate frame

protected:
  std::vector<std::string> coordinates; // a string name for each element in the vector (size==dim)
};

typedef std::shared_ptr<CoordinateFrame> CoordinateFramePtr;

class DLLEXPORT MultiCoordinateFrame : public CoordinateFrame {
public:
  MultiCoordinateFrame(const std::string& name, std::initializer_list<std::shared_ptr<CoordinateFrame>> _frames);
  virtual ~MultiCoordinateFrame(void) {};

  virtual std::ostream& print(std::ostream& os) const {
    os << "Multi-Coordinate Frame: " << name << " (" << getDim() << " elements)" << std::endl;

    for (auto sf : frames) {
      os << "  " << sf.frame->name << " (" << sf.frame->getDim() << " elements) " << std::endl;
    }
    return os;
  }

  virtual const unsigned int getDim() const { return coordinate_refs.size(); };
  virtual const std::string& getCoordinateName(unsigned int i) const {
    if (i>=coordinate_refs.size())
      throw std::runtime_error("index exceeds dimension of coordinate frame");
    return coordinate_refs[i].frame->getCoordinateName(coordinate_refs[i].index_in_subframe);
  }
  virtual std::vector<std::string> getCoordinateNames() const {
    std::vector<std::string> coordinates;
    for (auto c : coordinate_refs) {
      coordinates.push_back(c.frame->getCoordinateName(c.index_in_subframe));
    }
    return coordinates;
  }
  virtual void setCoordinateNames(const std::vector<std::string>& _coordinates)
  {
    throw std::runtime_error("cannot change the names of a multi-coordinate frame directory.  change the subframes.");
    // note: i could potentially allow this if the sizes were correct, but it seems unlikely to be used for good
  }

  virtual DrakeSystemPtr setupLCMOutputs(DrakeSystemPtr sys) { throw std::runtime_error("not implemented yet (need CoordinateTransforms)"); }

private:
  /// Store the data in two (compatible) ways:
  ///   1) as a list of subframes, with a map to the relevant indices in the multi-frame
  ///   2) as a list of coordinates, with references to the associated subframe
  struct SubFrame {
    std::shared_ptr<CoordinateFrame> frame;
    std::vector<unsigned int> coordinate_indices; // which indices in the multi-frame are associated with this sub-frame
  };
  struct CoordinateRef {
    CoordinateRef(const std::shared_ptr<CoordinateFrame>& _frame) : frame(_frame) {};
    const std::shared_ptr<CoordinateFrame>& frame;  // reference to one of the SubFrame shared pointers
    unsigned int index_in_subframe;
  };
  std::vector<struct SubFrame> frames;
  std::vector<struct CoordinateRef> coordinate_refs;
};


#endif // #define __CoordinateFrame_H_