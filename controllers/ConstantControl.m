classdef ConstantControl < RobotLibSystem
% Trivial control class that always outputs a constant u.
  
  methods
    function obj = ConstantControl(const_y)
      % Create ConstantControl object by specifying y.
      obj = obj@RobotLibSystem(0,0,0,length(const_y),false,true);
      obj.const_y = const_y;
    end
    function y = output(obj,t,x,u)
      % Implement control.
      y = obj.const_y;
    end
  end
  
  properties
    const_y;
  end
  
end