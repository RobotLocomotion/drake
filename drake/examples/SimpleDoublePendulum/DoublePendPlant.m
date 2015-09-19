classdef DoublePendPlant < Manipulator
  
  properties
    % parameters from Spong95 (except inertias are now relative to the
    % joint axes)
    l1 = 1; l2 = 1;  
    m1 = 1; m2 = 1;  
    g = 9.81;
    b1=.1;  b2=.1;
    I1=[]; I2=[]; % set in constructor
  end
  
  methods
    function obj = DoublePendPlant
      obj = obj@Manipulator(2,2);
      obj.I1 = obj.m1*obj.l1^2;
      obj.I2 = obj.m2*obj.l2^2;
      obj = setStateFrame(obj,CoordinateFrame('DoublePendState',4,'x',{'theta1','theta2','theta1_dot','theta2_dot'}));
      obj = obj.setOutputFrame(obj.getStateFrame);
    end
    
    function [H,C,B] = manipulatorDynamics(obj,q,qd)
      % keep it readable:
      m1=obj.m1; m2=obj.m2; l1=obj.l1; l2=obj.l2; g=obj.g; b1=obj.b1; b2=obj.b2; I1=obj.I1; I2=obj.I2;
      m2l1l2 = m2*l1*l2;  % occurs often!

      c = cos(q(1:2,:));  s = sin(q(1:2,:));  s12 = sin(q(1,:)+q(2,:));
      
      h12 = I2 + m2l1l2*c(2);
      H = [ I1 + I2 + m2*l1^2 + 2*m2l1l2*c(2), h12; h12, I2 ];
      
      C = [ -2*m2l1l2*s(2)*qd(2), -m2l1l2*s(2)*qd(2); m2l1l2*s(2)*qd(1), 0 ];
      G = g*[ m1*l1*s(1) + m2*(l1*s(1)+l2*s12); m2*l2*s12 ];
            
      % accumate total C and add a damping term:
      C = C*qd + G + [b1;b2].*qd;

      B = eye(2);
    end
    
    function x = getInitialState(obj)
      x = .1*randn(4,1);
    end
    
  end
  
  methods(Static)
    function run()  % runs the passive system
      pd = DoublePendPlant;
      pv = DoublePendVisualizer(pd);
      traj = simulate(pd,[0 5],randn(4,1));
      playback(pv,traj);
    end
  end
  
end
