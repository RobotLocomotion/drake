classdef PendulumEnergyShaping < SmoothRobotLibSystem
  
  properties 
    p
  end
  
  methods
    function obj = PendulumEnergyShaping(plant)
      obj = obj@SmoothRobotLibSystem(0,0,2,1,true,true);
      typecheck(plant,'PendulumPlant');
      obj.p = plant;
    end
    
    function u = output(obj,t,junk,x)
      Etilde = .5*obj.p.m*obj.p.l^2*x(2)^2 - obj.p.m*obj.p.g*obj.p.l*cos(x(1)) - 1.2*obj.p.m*obj.p.g*obj.p.l;
      u = +obj.p.b*x(2)-.1*x(2)*Etilde;
    end
  end 
  
end