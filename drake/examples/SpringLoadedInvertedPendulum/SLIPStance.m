classdef SLIPStance < DrakeSystem

  properties
    m;
    r0;
    k;
    g;
  end
    
  methods
    function obj=SLIPStance(slip)
      typecheck(slip,'SLIP');
      
      obj=obj@DrakeSystem(5,0,1,8,false,true);
      obj.m = slip.m;
      obj.r0 = slip.r0;
      obj.k = slip.k;
      obj.g = slip.g;
      
      obj = setStateFrame(obj,CoordinateFrame('SLIPStanceState',5,'x',{'r','theta','rdot','thetadot','x_toe'}));
      
      obj = setInputFrame(obj,getInputFrame(slip));
      obj = setOutputFrame(obj,getOutputFrame(slip));
    end
        
    function [xdot,df]=dynamics(obj,t,x,u)
      rddot=obj.k/obj.m*(obj.r0-x(1))+x(1)*x(4)^2-obj.g*cos(x(2));
      thetaddot=(obj.g*x(1)*sin(x(2))-2*x(1)*x(3)*x(4))/(x(1)^2);
      xdot=[x(3:4);rddot;thetaddot;0];
      if(nargout>1)
        dfdx=zeros(5);
        dfdx(1:2,3:4)=eye(2);
        dfdx(3,1)=-obj.k/obj.m+x(4)^2;
        dfdx(3,2)=obj.g*sin(x(2));
        dfdx(3,4)=2*x(1)*x(4);
        dfdx(4,1) = -(obj.g*sin(x(2))-2*x(3)*x(4))/x(1)^2;
        dfdx(4,2) = obj.g*cos(x(2))/x(1);
        dfdx(4,3) = -2*x(4)/x(1);
        dfdx(4,4) = -2*x(3)/x(1);
        df=[zeros(5,1) dfdx zeros(5,1)];
      end
    end
        
    function y=output(obj,t,x,u)
      r=x(1);theta=x(2);x_toe=x(5);
      rdot=x(3);thetadot=x(4);
      hip = [x_toe;0]+r*[-sin(theta);cos(theta)];
      hipdot = rdot*[-sin(theta);cos(theta)] - r*[cos(theta);sin(theta)];
      y=[hip;r;theta;hipdot;rdot;thetadot];
    end
  end
end