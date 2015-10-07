classdef ConstrainedPlant < Manipulator
  
  methods
    function obj = ConstrainedPlant()
      obj = obj@Manipulator(2,1,2);
      
      fun_handle = DrakeFunctionHandle(2,1,@obj.constraintFun);
      con = DrakeFunctionConstraint(0,0,fun_handle);
      con.grad_level = 2;
      obj = obj.addPositionEqualityConstraint(con);      
    end
    
    function [f,df,ddf] = constraintFun(obj,q)
      f = q(1)^2 + 2*q(2)^2 - 3*q(1)*q(2) + 2*q(1) - q(2);
      df = [2*q(1)-3*q(2)+2, 4*q(2)-3*q(1)-1];
      ddf = [2,-3,-3,4];
      
%       f = q(1) + cos(q(2)) - 1;
%       df = [1 -sin(q(2))];
%       ddf = [0 0 0 -cos(q(2))];
    end
    
    function [H,C,B,dH,dC,dB] = manipulatorDynamics(obj,q,v)
      H = [1 .2; .2 1];
      C = -q;
      B = zeros(2,1);
      
      dH = zeros(4,4);
      dC = [-eye(2) zeros(2)];
      dB = zeros(2,4);
    end
    
    function fr = getVelocityFrame(obj)
      fr = CoordinateFrame('velocity',2);
    end
  end
  
end