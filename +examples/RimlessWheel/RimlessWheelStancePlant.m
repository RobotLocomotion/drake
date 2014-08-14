classdef RimlessWheelStancePlant < DrakeSystem 

  properties (SetAccess=private)
    m, l, g
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
    
    function xdot = dynamics(obj,t,x,u)
      xdot = [x(2,:); (obj.m*obj.g*obj.l*sin(x(1,:)))/(obj.m*obj.l^2); 0];
    end
  
    function df = dynamics_gradients(obj,t,x,u,order)
      if (nargin<5) order=1;
      elseif (order>1) error('not implemented yet'); end
      df{1} = [zeros(3,1), [0, 1, 0; obj.g*cos(x(1))/obj.l, 0, 0; zeros(1,3)]];
    end
    
    function y = output(obj,t,x,u)
      y = x;
    end
    
    function x0 = getInitialState(obj)
      x0 = [0.1*randn; 20*randn; 0];
    end
    
  end
  
  
  
end
