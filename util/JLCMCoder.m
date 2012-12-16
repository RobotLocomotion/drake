classdef JLCMCoder < LCMCoder
  
  methods
    function obj=JLCMCoder(jcoder)
      obj.jcoder=jcoder;
    end
    
    function str = timestampName(obj)
      str = obj.jcoder.timestampName();
    end
    
    function [x,t] = decode(obj,data)
      fd = obj.jcoder.decode(data);
      x = fd.val;
      t = fd.t;
    end
    
    function msg = encode(obj,t,x)
      msg = obj.jcoder.encode(drake.util.CoordinateFrameData(t,x));
    end
    
  end
  
  properties
    jcoder
  end
  
end