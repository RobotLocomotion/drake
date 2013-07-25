classdef SimpleMixedCTDTExample < DrakeSystem
  methods
    function obj = SimpleMixedCTDTExample()
      % call the parent class constructor:
      obj = obj@DrakeSystem(...  
         1, ... % number of continuous states
         1, ... % number of discrete states
         0, ... % number of inputs
         2, ... % number of outputs
         false, ... % because the output does not depend on u
         true);  % because the update and output do not depend on t
    end
    function ts = getSampleTime(obj)
      ts = [[0;0], ...  % continuous and discrete sample times
        [1;0]];         % with dt = 1
    end
    function xdnext = update(obj,t,x,u)
      xdnext = x(1)^3;       % the DT state is x(1)
    end
    function xcdot = dynamics(obj,t,x,u);
      xcdot = -x(2)+x(2)^3;  % the CT state is x(2)
    end
    function y=output(obj,t,x,u)
      y=x;
    end
  end
end
