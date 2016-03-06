classdef RimlessWheelStancePlant < DrakeSystem 

  properties (SetAccess=private)
    m=1, l=1, g=9.81
  end
  
  methods 
    function obj = RimlessWheelStancePlant(m,g,l)
      obj = obj@DrakeSystem(3,0,0,3,false,true); % [theta; thetadot; position of toe] 
      if (nargin>0)
        obj.m = m;
        obj.g = g;
        obj.l = l;
      end
%      obj = obj.setInputLimits(-3,3);
    end
    
    function [xdot,df] = dynamics(obj,t,x,u)
      xdot = [x(2,:); obj.g*sin(x(1,:))/obj.l; 0*x(3,:)];
      if nargout>1,
        df = [zeros(3,1), [0, 1, 0; obj.g*cos(x(1))/obj.l, 0, 0; zeros(1,3)]];
      end
    end
    
    function thetadot = orbit(obj,theta,E)
      if nargin<3, E = obj.m*obj.g*obj.l; end % homoclinic
      thetadot = sqrt(2*(E - obj.m*obj.g*obj.l*cos(theta))./(obj.m*obj.l^2));
    end
      
    function [y,dy] = output(obj,t,x,u)
      y = x;
      dy = [zeros(3,1),eye(3)];
    end
    
    function x0 = getInitialState(obj)
      x0 = [0.1*randn; 20*randn; 0];
    end
    
  end
  
  
  
end
