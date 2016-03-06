classdef LCMCoder
% An LCM coder translates an LCM type into a vector of doubles used in
% Drake.  This is the interface class for writing LCM coders in matlab.
% Note that there is also an LCMCoder.java which is the interface class
% (which provides the same interface) for authoring coders in java.
  
  methods (Abstract)
    d = dim(obj);
    str = timestampName(obj);
    names = coordinateNames(obj);
    [x,t] = decode(obj,data);
    msg = encode(obj,t,x);
  end
end
  