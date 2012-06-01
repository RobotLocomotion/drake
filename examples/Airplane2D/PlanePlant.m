classdef PlanePlant < DrakeSystem
% Defines the dynamics for the powered plane.
  
  properties
    m = 0.082;
    g = 9.81; % m/s^2
  end
  
  methods
    function obj = PlanePlant()
      obj = obj@DrakeSystem(4,0,1,4,0,1);
    end
    
    function [xdot, df, d2f, d3f] = dynamics(obj,t,x,u)
        % x = [ x; y; theta; thetadot ]
        % u = [ thetadotdot ]
        
        theta = x(3);
        v = 10;
        xdot = [ -v * sin(theta);  v * cos(theta); x(4); u(1) ];
        
        if (nargout>1)
            [df,d2f,d3f]= dynamicsGradients(obj,t,x,u,nargout-1);
        end
    end
    
    function y = output(obj,t,x,u)
      y = x;
    end
    
    function x = getInitialState(obj)
      x = [0 0 0 0]';
    end
    
  end
  
end
