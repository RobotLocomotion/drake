classdef ConstrainedPlant < Manipulator
  
  methods
    function obj = ConstrainedPlant()
      obj = obj@Manipulator(2,1,2);
      
      fun_handle = drakeFunction.DrakeFunctionHandle(2,1,@obj.constraintFun);
      con = DrakeFunctionConstraint(0,0,TestConstraint());
      con.grad_level = 2;
      obj = obj.addPositionEqualityConstraint(con);      
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