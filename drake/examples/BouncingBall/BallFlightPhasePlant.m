classdef BallFlightPhasePlant < DrakeSystem
  
  methods
    function obj = BallFlightPhasePlant()
      obj = obj@DrakeSystem(...
        2, ... % number of continuous states
        0, ... % number of discrete states
        0, ... % number of inputs
        1, ... % number of outputs
        false, ... % not direct feedthrough
        true); % time invariant
    end
    
    function xdot = dynamics(obj,t,x,u)
      xdot = [x(2); -obj.g];  % qddot = -g; x=[q,qdot]
    end
    
    function y = output(obj,t,x,u)
      y = x(1); % height of the ball
    end
  end

  properties
    g = 9.81;  % gravity
  end
end
