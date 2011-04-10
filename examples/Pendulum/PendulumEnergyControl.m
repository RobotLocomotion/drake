classdef PendulumEnergyControl < HybridRobotLibSystem
  
  methods
    function obj = PendulumEnergyControl(plant)
      obj = obj.addMode(PendulumEnergyShaping(plant));
      [lqr,V] = PendulumLQR(plant);
      obj = obj.addMode(lqr);
      obj = obj.setModeOutputFlag(false);
      obj = obj.setAngleFlags([1;0],[],0);

      % todo: build closed-loop lqr system and do verification on it to
      % determine transition boundary.
      obj.x0 = lqr.x0;
      obj.S = lqr.S;
      obj.rho = 4;

      in_lqr_roa = inline('obj.wrapInput(x-obj.x0)''*obj.S*obj.wrapInput(x-obj.x0) - obj.rho','obj','t','junk','x');
      
      obj = obj.addTransition(1,in_lqr_roa,@transitionToLQR,true,true);
      obj = obj.addTransition(2,notGuard(obj,in_lqr_roa),@transitionFromLQR,true,true);
      
    end
    
    function [x,m,status] = transitionToLQR(obj,m,t,x,u)
      status=0;
      m=2;
    end
    
    function [x,m,status] = transitionFromLQR(obj,m,t,x,u)
      status=0;
      m=1;
    end
    
  end
  
  methods (Static)
    function run()
      pd = PendulumPlant;
      pv = PendulumVisualizer;
      c = PendulumEnergyControl(pd);

      sys = feedback(pd,c);

      for i=1:5
        xtraj = simulate(sys,[0 6]);
        playback(pv,xtraj);
      end
    end
  end
  
  properties
    x0
    S
    rho
  end
  
end
