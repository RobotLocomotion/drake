classdef CartPolePlant < Manipulator

  properties
    mc = 10;   % mass of the cart in kg
    mp = 1;    % mass of the pole (point mass at the end) in kg
    l = 0.5;   % length of the pole in m
    g = 9.81;  % gravity m/s^2
  end
  
  methods
    function obj = CartPolePlant
      obj = obj@Manipulator(2,1);
      obj = setInputLimits(obj,-30,30);
%      obj = setAngleFlags(obj,0,[0;1;0;0],[0;1;0;0]);
    end
        
    function [H,C,B] = manipulatorDynamics(obj,q,qd)
      mc=obj.mc;  mp=obj.mp;  l=obj.l;  g=obj.g;
      s = sin(q(2)); c = cos(q(2));

      H = [mc+mp, mp*l*c; mp*l*c, mp*l^2];
      C = [0 -mp*qd(2)*l*s; 0 0];
      G = [0; mp*g*l*s];
      B = [1; 0];
      
      C = C*qd + G;
    end
    
    function [f,df,d2f,d3f] = dynamics(obj,t,x,u)
      f = dynamics@Manipulator(obj,t,x,u);
      if (nargout>1)
        [df,d2f,d3f]= dynamicsGradients(obj,t,x,u,nargout-1);
      end
    end
    
    function x0 = getInitialState(obj)
      x0 = randn(4,1);
    end
    
  end
  
  methods (Static)
    function run()
      d = CartPolePlant;
      v = CartPoleVisualizer(d);
      xtraj = simulate(d,[0 5]);
      playback(v,xtraj);    
    end
  end
  
end
