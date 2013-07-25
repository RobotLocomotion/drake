classdef MassSpringDamperPlant < SecondOrderSystem
% Defines the dynamics for the Pendulum.
  
  properties
    m = 1;   % kg
    b = .5;
    k = 20;
  end
  
  methods
    function obj = MassSpringDamperPlant()
      % Construct a new PendulumPlant
      obj = obj@SecondOrderSystem(1,1,true);
      obj = setOutputFrame(obj,obj.getStateFrame);
    end
    
    function qdd = sodynamics(obj,t,q,qd,u)
      % Implement the second-order dynamics
      qdd = (u - obj.k*q - obj.b*qd)/obj.m;
    end
    
    function x = getInitialState(obj)
      % Start me anywhere!
      x = randn(2,1);
    end
  end  

end
