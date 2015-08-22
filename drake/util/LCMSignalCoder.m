classdef LCMSignalCoder < LCMCoder
  % handle the lcmt_drake_signal type
  
  methods 
    function obj = LCMSignalCoder(coordinate_names)
      obj.coordinate_names = coordinate_names;
    end
    function d = dim(obj)
      d = length(obj.coordinate_names);
    end
    function str = timestampName(obj)
      str = 'timestamp';
    end
    function names = coordinateNames(obj)
      names=obj.coordinate_names;
    end
    function [x,t] = decode(obj,data)
      msg = drake.lcmt_drake_signal(data);
      t = msg.timestamp/1000;
      x = msg.val;  % todo: would be more robust to string match the coordinate names
    end
    function msg = encode(obj,t,x)
      msg = drake.lcmt_drake_signal();
      msg.timestamp = t*1000;
      msg.dim = numel(x);
      msg.coord = obj.coordinate_names;
      msg.val = x;
    end
  end
  
  properties
    coordinate_names;
  end
end