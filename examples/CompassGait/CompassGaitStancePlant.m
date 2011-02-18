classdef CompassGaitStancePlant < ManipulatorPlant 

  properties
    m, mh, a, b, l, g;
  end
  
  methods 
    function obj = CompassGaitStancePlant(m,mh,a,b,l,g)
      obj = obj@ManipulatorPlant(2,1);
      if (nargin>0)
        obj.m=m; obj.mh=mh; obj.a=a; obj.b=b; obj.l=l; obj.g=g;
      end
      
      obj = obj.setInputLimits(-inf,inf);
    end
    
    function [H,C,B] = manipulatorDynamics(obj,q,qdot)
      m=obj.m; l=obj.l; b=obj.b; mh=obj.mh; a=obj.a; g=obj.g;
      H = [ m*b^2, -m*l*b*cos(q(2)-q(1)); -m*l*b*cos(q(2)-q(1)), (mh+m)*l^2 + m*a^2];
      C = [0, m*l*b*sin(q(2)-q(1))*qdot(2); m*l*b*sin(q(1)-q(2))*qdot(1), 0];
      G = g*[ m*b*sin(q(1)); -(mh*l + m*a + m*l)*sin(q(2)) ];
      C=C*qdot+G;
      B = [-1;1];
    end
    
    function df = dynamicsGradients(obj,t,x,u,order)
      if (nargin<5) order=1; end
      df = compassGaitGradients(obj,t,x,u,order);
    end
    
    function x0 = getInitialState(obj)
      x0 = [0; 0; 2.0; -0.4];
    end
    
  end
  
end