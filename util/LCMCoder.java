package drake.util;

import lcm.lcm.*;

public interface LCMCoder {
  String timestampName();
  CoordinateFrameData decode(byte[] data);
  LCMEncodable encode(CoordinateFrameData d);
}