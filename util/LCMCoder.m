classdef LCMCoder
  
  methods (Abstract)
    str = timestampName();
    [x,t] = decode(obj,data);
    msg = encode(obj,t,x);
  end
end
  