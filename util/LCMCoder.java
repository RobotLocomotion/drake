package drake.util;

import lcm.lcm.*;

public interface LCMCoder {
  String timestampName();
  drake.util.CoordinateFrameData decode(byte[] data);
  LCMEncodable encode(drake.util.CoordinateFrameData d);
}