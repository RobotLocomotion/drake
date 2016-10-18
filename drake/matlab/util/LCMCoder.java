package drake.matlab.util;

import lcm.lcm.*;

// An LCM coder translates an LCM type into a vector of doubles used in
// Drake.  This is the interface class for writing LCM coders in java. 
// Note that there is also an LCMCoder.m which is the interface class
// (which provides the same interface) for authoring coders in matlab.


public interface LCMCoder {
  int dim();  // returns the dimension of the vector being encoded/decoded 
  String timestampName();
  String[] coordinateNames();
  drake.matlab.util.CoordinateFrameData decode(byte[] data);
  LCMEncodable encode(drake.matlab.util.CoordinateFrameData d);
}
