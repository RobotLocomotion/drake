classdef SimpleDTExample < DrakeSystem
  methods
    function obj = SimpleDTExample()
      % call the parent class constructor:
      obj = obj@DrakeSystem(...  
         0, ... % number of continuous states
         1, ... % number of discrete states
         0, ... % number of inputs
         1, ... % number of outputs
         false, ... % because the output does not depend on u
         true);  % because the update and output do not depend on t
    end
    function xnext = update(obj,t,x,u)
      xnext = x^3;
    end
    function y=output(obj,t,x,u)
      y=x;
    end
  end
end
