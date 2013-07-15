classdef SimpleCTExample < DrakeSystem
  methods
    function obj = SimpleCTExample()
      % call the parent class constructor:
      obj = obj@DrakeSystem(...  
         1, ... % number of continuous states
         0, ... % number of discrete states
         0, ... % number of inputs
         1, ... % number of outputs
         false, ... % because the output does not depend on u
         true);  % because the dynamics and output do not depend on t
    end
    function xdot = dynamics(obj,t,x,u)
      xdot = -x+x^3;
    end
    function y=output(obj,t,x,u)
      y=x;
    end
  end
end
