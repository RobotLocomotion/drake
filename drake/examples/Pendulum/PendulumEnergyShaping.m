classdef PendulumEnergyShaping < DrakeSystem
  
  properties 
    p
  end
  
  methods
    function obj = PendulumEnergyShaping(plant)
      obj = obj@DrakeSystem(0,0,2,1,true,true);
      typecheck(plant,'PendulumPlant');
      obj.p = plant;
      obj = setInputFrame(obj,PendulumState);
      obj = setOutputFrame(obj,PendulumInput);
    end
    
    function u = output(obj,t,junk,x)
      Etilde = .5*obj.p.m*obj.p.l^2*x(2)^2 - obj.p.m*obj.p.g*obj.p.l*cos(x(1)) - 1.1*obj.p.m*obj.p.g*obj.p.l;
      u = +obj.p.b*x(2)-.1*x(2)*Etilde;
    end
  end 
  
end
