classdef ConstantControl < SmoothRobotLibSystem
% Trivial control class that always outputs a constant u.
  
  methods
    function obj = ConstantControl(const_y,num_inputs)
      % Create ConstantControl object by specifying y.
      % @param num_inputs optional number of inputs (to ignore).  @default 0
      obj = obj@SmoothRobotLibSystem(0,0,0,length(const_y),false,true);
      obj.const_y = const_y;
      if (nargin>1) obj = setNumInputs(obj,num_inputs); end
    end
    function y = output(obj,~,~,~)
      % Implement control.
      y = obj.const_y;
    end
  end
  
  properties
    const_y;
  end
  
end
