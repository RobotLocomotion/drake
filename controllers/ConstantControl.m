classdef ConstantControl < Control
  
  methods
    function obj = ConstantControl(const_u)
      obj = obj@Control(0,length(const_u));
      obj.const_u = const_u;
    end
    function u = control(obj,t,x)
      u = obj.const_u;
    end
  end
  
  properties
    const_u;
  end
  
end