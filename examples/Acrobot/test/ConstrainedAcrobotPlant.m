classdef ConstrainedAcrobotPlant < Manipulator
% Simple plant to test the bilateral constraints implementation in
% ManipulatorPlant

  properties
    l1 = 1; l2 = 2;  
    m1 = 1; m2 = 1;  
    g = 9.81;
    b1=.1;  b2=.1;
    lc1 = .5; lc2 = 1; 
    I1=[]; I2 = [];  % set in constructor
  end
  
  methods
    function obj = ConstrainedAcrobotPlant
      obj = obj@Manipulator(2,1,1);
      obj = setInputLimits(obj,-10,10);
      obj.I1 = 0.083 + obj.m1*obj.lc1^2;
      obj.I2 = 0.33 + obj.m2*obj.lc2^2;
    end
    
    function [H,C,B] = manipulatorDynamics(obj,q,qd)
      % keep it readable:
      m1=obj.m1; m2=obj.m2; l1=obj.l1; g=obj.g; lc1=obj.lc1; lc2=obj.lc2; I1=obj.I1; I2=obj.I2; b1=obj.b1; b2=obj.b2;
      m2l1lc2 = m2*l1*lc2;  % occurs often!

      c = cos(q(1:2,:));  s = sin(q(1:2,:));  s12 = sin(q(1,:)+q(2,:));
      
      h12 = I2 + m2l1lc2*c(2);
      H = [ I1 + I2 + m2*l1^2 + 2*m2l1lc2*c(2), h12; h12, I2 ];
      
      C = [ -2*m2l1lc2*s(2)*qd(2), -m2l1lc2*s(2)*qd(2); m2l1lc2*s(2)*qd(1), 0 ];
      G = g*[ m1*lc1*s(1) + m2*(l1*s(1)+lc2*s12); m2*lc2*s12 ];
            
      % accumate total C and add a damping term:
      C = C*qd + G + [b1;b2].*qd;

      B = [0; 1];
    end

    function phi = bilateralConstraints(obj,q)
      phi = q(1);  % constrain first link to zero
    end
    
    function x = getInitialState(obj)
      x = .1*randn(4,1);
      x([1,3]) = 0;  % to satisfy the constraints
    end
    
  end
end
