classdef BallFlightPhasePlant2D < DrakeSystem
  
  methods
    function obj = BallFlightPhasePlant2D()
      obj = obj@DrakeSystem(...
        4, ... % number of continuous states
        0, ... % number of discrete states
        0, ... % number of inputs
        2, ... % number of outputs
        false, ... % not direct feedthrough
        true); % time invariant
    end
    
    function xdot = dynamics(obj,t,x,u)
      xdot = [x(3:4); 0; -obj.g];  % qddot = [0; -g]; x=[q,qdot]
    end
    
    function y = output(obj,t,x,u)
      y = x(1:2); % horizontal and vertical position of the ball
    end
  end

  properties
    g = 9.81;  % gravity
  end
end
