
#include "CoordinateFrame.h"

MultiCoordinateFrame::MultiCoordinateFrame(const std::string& name, std::initializer_list<std::shared_ptr<CoordinateFrame>> _frames)
  : CoordinateFrame(name) {
  unsigned int dim = 0;
  for (auto subframe : _frames) {
    if (!subframe) continue;  // ok if they pass in nullptr
    struct SubFrame s;
    s.frame = subframe;
    for (unsigned int i=0; i<subframe->getDim(); i++) {
      s.coordinate_indices.push_back(dim+i);

      struct CoordinateRef c(s.frame);
      c.index_in_subframe = i;
      coordinate_refs.push_back(c);

      // just to get the dim right, because the subframes could change out from underneath us (via some other reference)
      // instead, I overload all of the access methods that look for coordinates
      coordinates.push_back("NEVER USE");
    }
    dim += subframe->getDim();
    frames.push_back(s);
  }
}
