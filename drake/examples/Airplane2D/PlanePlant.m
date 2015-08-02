classdef PlanePlant < DrakeSystem
% Defines the dynamics for the powered plane.
  
  properties
    v = 10;  % m/s (fixed forward speed)
    m = 0.082;
    g = 9.81; % m/s^2
  end
  
  methods
    function obj = PlanePlant()
      obj = obj@DrakeSystem(4,0,1,4,0,1);
      obj = setStateFrame(obj,CoordinateFrame('PlaneState',4,'x',{'x','y','theta','thetadot'}));
      obj = setInputFrame(obj,CoordinateFrame('PlaneInput',1,'u',{'thetaddot'}));
      obj = setOutputFrame(obj,getStateFrame(obj));  % allow full state feedback
    end
    
    function [xdot, df, d2f, d3f] = dynamics(obj,t,x,u)
        % x = [ x; y; theta; thetadot ]
        % u = [ thetaddot ]
        
        theta = x(3);
        xdot = [ -obj.v * sin(theta);  obj.v * cos(theta); x(4); u(1) ];
        
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
