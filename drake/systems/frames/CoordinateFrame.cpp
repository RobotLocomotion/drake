
#include "CoordinateFrame.h"

MultiCoordinateFrame::MultiCoordinateFrame(const std::string& name, std::initializer_list<std::shared_ptr<CoordinateFrame>> _frames)
  : CoordinateFrame(name) {
  unsigned int dim = 0;
  for (auto subframe : _frames) {
    if (!subframe) continue;  // ok if they pass in nullptr
    struct SubFrame s;
    s.frame = subframe;
    struct CoordinateRef c;
    c.frame = subframe;

    for (unsigned int i=0; i<subframe->getDim(); i++) {
      s.coordinate_indices.push_back(dim+i);
      c.index_in_subframe = i;
      coordinates.push_back(subframe->getCoordinateName(i));
    }
    dim += subframe->getDim();
    frames.push_back(s);
    coordinate_refs.push_back(c);
  }
}
