classdef SLIPFlight < DrakeSystem

  properties
    r0
    g
  end
  methods
    function obj=SLIPFlight(slip)
      typecheck(slip,'SLIP');
      
      obj=obj@DrakeSystem(4,0,1,8,true,true);
      obj.r0 = slip.r0;
      obj.g = slip.g;
      
      obj = setStateFrame(obj,CoordinateFrame('SLIPFlightState',4,'x',{'x','y','xdot','ydot'}));
      
      obj = setInputFrame(obj,getInputFrame(slip));
      obj = setOutputFrame(obj,getOutputFrame(slip));
    end
    
    function  x0=getInitialState(obj)
      x0=[0;1.5;5;0];
    end
        
%         function obj=setInitialState(obj,y0)
%             obj.x0=[0;y0;sqrt(2*(obj.E-obj.m*obj.g*y0)/obj.m);0];
%         end

    function [xdot,df]=dynamics(obj,t,x,u)
      xdot=[x(3:4);0;-obj.g];
      if(nargout>1)
        dfdx=[zeros(2) eye(2);zeros(2,4)];
        df=[zeros(4,1) dfdx zeros(4,1)];
      end
    end
    
    function y=output(obj,t,x,u)
      y=[x(1:2);obj.r0;u;x(3:4);0;0];
    end
    
  end
end