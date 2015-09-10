
classdef LinearGaussianExample < StochasticDrakeSystem

  methods
    function obj = LinearGaussianExample
      obj = obj@StochasticDrakeSystem(...
        1, ... % number of continuous states
        0, ... % number of discrete states
        0, ... % number of inputs
        1, ... % number of outputs
        false, ...  % not direct feedthrough
        true, ...   % time invariant
        1, ... % number of noise inputs
        .01);  % time constant of w(t)
    end

    function xcdot = stochasticDynamics(obj,t,x,u,w)
      xcdot = -x + w;
    end

    function y = stochasticOutput(obj,t,x,u,w);
      y=x;
    end
  end
end

