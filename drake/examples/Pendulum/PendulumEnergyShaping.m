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
    
    function y = output(obj,t,~,u)
      Etilde = .5*obj.p.m*obj.p.l^2*u(2)^2 - obj.p.m*obj.p.g*obj.p.l*cos(u(1)) - 1.1*obj.p.m*obj.p.g*obj.p.l;
      y = +obj.p.b*u(2)-.01*u(2)*Etilde;
    end
  end 

  methods (Static)
    function run()
      pd = PendulumPlant;
      pd = setInputLimits(pd,-inf,inf);
      pv = PendulumVisualizer();
      c = PendulumEnergyShaping(pd);

      sys = feedback(pd,c);
      
      for i=1:5
        xtraj = simulate(sys,[0 6]);
        playback(pv,xtraj);
        figure(2); fnplt(xtraj);
      end
    end
  end  
end
