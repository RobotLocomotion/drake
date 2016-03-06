classdef CompassGaitStancePlant < Manipulator

  properties
    m = 5;
    mh = 10;
    a = 0.5;
    b = 0.5;
    g = 9.8;
    l;
  end
  
  methods 
    function obj = CompassGaitStancePlant(m,mh,a,b,l,g)
      obj = obj@Manipulator(2,1);
      obj.l=obj.a+obj.b;
      if (nargin>0)
        obj.m=m; obj.mh=mh; obj.a=a; obj.b=b; obj.l=l; obj.g=g;
      end
      
      obj = obj.setInputLimits(-50,50);
      obj = setOutputFrame(obj,getStateFrame(obj));
    end
    
    function [H,C,B] = manipulatorDynamics(obj,q,qdot)
      m=obj.m; l=obj.l; b=obj.b; mh=obj.mh; a=obj.a; g=obj.g;
      H = [ m*b^2, -m*l*b*cos(q(2)-q(1)); -m*l*b*cos(q(2)-q(1)), (mh+m)*l^2 + m*a^2];
      C = [0, m*l*b*sin(q(2)-q(1))*qdot(2); m*l*b*sin(q(1)-q(2))*qdot(1), 0];
      G = g*[ m*b*sin(q(1)); -(mh*l + m*a + m*l)*sin(q(2)) ];
      C=C*qdot+G;
      B = [-1;1];
    end

    function [f,df] = dynamics(obj,t,x,u)
      f = dynamics@Manipulator(obj,t,x,u);
      if (nargout>1)
        df= dynamicsGradients(obj,t,x,u,nargout-1);
      end
    end
    
    function x0 = getInitialState(obj)
      x0 = [0; 0; 2.0; -0.4];
    end
    
        
    function n = getNumPositions(obj)
      n = 2;
    end
    function n = getNumVelocities(obj)
      n = 2;
    end
    
  end
  
end
