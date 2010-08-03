classdef PendulumEnergyShaping < Control
  
  properties 
    p
    lqr;
  end
  
  methods
    function obj = PendulumEnergyShaping(dyn)
      obj = obj@Control(2,1);
      obj.p = dyn;
      obj.lqr = PendulumLQR(dyn);
      obj.control_dt = 0;
    end
    
    function u = control(obj,t,x)
      if obj.lqr.isVerified(x)
        u = obj.lqr.control(t,x);
      else
        Etilde = .5*obj.p.m*obj.p.l^2*x(2)^2 - obj.p.m*obj.p.g*obj.p.l*cos(x(1)) - 1.2*obj.p.m*obj.p.g*obj.p.l;
        u = -.1*x(2)*Etilde;
      end
    end
  end 
  
end