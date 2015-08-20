#ifndef __CoordinateFrame_H__
#define __CoordinateFrame_H__

#include <string>
#include <memory>
#include <vector>
#include <exception>
#include <initializer_list>

/// Every input, state, and output in a DynamicalSystem has a coordinate frame
/// attached to it.  Many bugs can be avoided by forcing developers to be
/// explicit about these coordinate systems when they make combinations of systems.

class CoordinateFrame {
public:
  CoordinateFrame(const std::string& _name, const unsigned int _dim, const std::vector<std::string>& _coordinates)
          : name(_name), dim(_dim), coordinates(_coordinates) {};
  CoordinateFrame(const std::string& _name, const unsigned int _dim, const std::string& prefix)
          : name(_name), dim(_dim) {
    for (int i=0; i<dim; i++) coordinates.push_back(prefix+std::to_string(i));
  }
  CoordinateFrame(const std::string& _name) : name(_name), dim(0) {}

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

protected:
  std::string name;  // a descriptive name for this coordinate frame
  unsigned int dim;  // number of elements in the coordinate vector
  std::vector<std::string> coordinates; // a string name for each element in the vector (size==dim)
};


class MultiCoordinateFrame : public CoordinateFrame {
public:
  MultiCoordinateFrame(const std::string& name, std::initializer_list<std::shared_ptr<CoordinateFrame>> _frames)
          : CoordinateFrame(name) {
    for (auto subframe : _frames) {
      struct SubFrame s;
      s.frame = subframe;
      struct CoordinateRef c;
      c.frame = subframe;

      for (unsigned int i=0; i<subframe->getDim(); i++) {
        s.coordinate_indices.push_back(dim+i);
        c.index_in_subframe = i;
        coordinates[i] = subframe->getCoordinateName(i);
      }
      dim += subframe->getDim();
      frames.push_back(s);
      coordinate_refs.push_back(c);
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