classdef JLCMCoder < LCMCoder
  
  methods
    function obj=JLCMCoder(jcoder)
      obj.jcoder=jcoder;
    end
    
    function d = dim(obj)
      d = obj.jcoder.dim();
    end
    
    function str = timestampName(obj)
      str = obj.jcoder.timestampName();
    end
    
    function str = coordinateNames(obj)
      str = cell(obj.jcoder.coordinateNames());
    end
    
    function [x,t] = decode(obj,data)
      fd = obj.jcoder.decode(data);
      x = fd.val;
      t = fd.t;
    end
    
    function msg = encode(obj,t,x,varargin)
      msg = obj.jcoder.encode(drake.matlab.util.CoordinateFrameData(t,x),varargin{:});
    end
    
  end
  
  properties
    jcoder
  end
  
end
