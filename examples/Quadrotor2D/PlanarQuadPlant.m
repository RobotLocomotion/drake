classdef PlanarQuadPlant < SecondOrderPlant

  % state:  
  %  q(1) - x position
  %  q(2) - z position
  %  q(3) - pitch (theta)
  % input:
  %  u(1) - prop 1 thrust
  %  u(2) - prop 2 thrust

  properties  %  based on (Bouadi, Bouchoucha, Tadjine 2007)
    L = 0.25; % length of rotor arm
    m = 0.486; % mass of quadrotor
    I = 0.00383; % moment of inertia
    g = 9.81; % gravity
  end
  
  methods
    function obj = PlanarQuadPlant()
      obj = obj@SecondOrderPlant(3,2,true);
    end
    
    function qdd = sodynamics(obj,t,q,qd,u)
      % Implement the second-order dynamics
      qdd = [ -sin(q(3))/obj.m*(u(1)+u(2));
        -obj.g + cos(q(3))/obj.m*(u(1)+u(2));
        obj.L/obj.I*(-u(1)+u(2))];
    end
    
    function x = getInitialState(obj)
      x = randn(6,1);
    end
    
  end  
  
end
