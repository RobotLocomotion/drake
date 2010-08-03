classdef ConstantControl < Control
% Trivial control class that always outputs a constant u.
  
  methods
    function obj = ConstantControl(const_u)
      % Create ConstantControl object by specifying u.
      obj = obj@Control(0,length(const_u));
      obj.const_u = const_u;
    end
    function u = control(obj,t,x)
      % Implement control.
      u = obj.const_u;
    end
  end
  
  properties
    const_u;
  end
  
end