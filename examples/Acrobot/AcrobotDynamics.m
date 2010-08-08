classdef AcrobotDynamics < ManipulatorDynamics 
  
  properties
    l1 = 1; l2 = 2;  
    m1 = 1; m2 = 1;  
    g = 9.81;
%    b1=.1;  b2=.1;
    b1=0; b2=0;
    lc1 = .5; lc2 = 1; 
    I1 = 1/3; I2 = 4/3; % solid rod m*l^2/3  % taken about the joint
  end
  
  methods
    function obj = AcrobotDynamics
      obj = obj@ManipulatorDynamics(2,1);
      obj = setInputLimits(obj,-3,3);
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
    
    % todo: also implement sodynamics here so that I can keep the
    % vectorized version?
    
    function df = dynamicsGradients(obj,t,x,u,order)
      if (nargin<5) order=1; end
      df = acrobotGradients(obj,t,x,u,order);
    end
    function x = getInitialState(obj)
      x = .1*randn(4,1);
    end
    
  end
end