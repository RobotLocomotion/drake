classdef PendulumEnergyControl < HybridRobotLibSystem
  
  methods
    function obj = PendulumEnergyControl(plant)
      obj = obj.addMode(PendulumEnergyShaping(plant));
      [obj.lqr,obj.V] = PendulumLQR(plant);
      obj = obj.addMode(obj.lqr);
      obj = obj.setModeOutputFlag(false);
      obj = obj.setAngleFlags([1;0],[],0);

%      in_lqr_roa = inline('obj.wrapInput(x-obj.x0)''*obj.S*obj.wrapInput(x-obj.x0) - obj.rho','obj','t','junk','x');
      p_x=decomp(obj.V);
      in_lqr_roa = @(p,t,junk,x) double(subs(obj.V,p_x,obj.wrapInput(x-obj.lqr.x0)+obj.lqr.x0))-1;
      
      obj = obj.addTransition(1,in_lqr_roa,@transitionToLQR,true,true);
      obj = obj.addTransition(2,notGuard(obj,in_lqr_roa),@transitionFromLQR,true,true);

      obj = setSimulinkParam(obj,'InitialStep','1e-3','MaxStep','0.05');
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
      
      figure(2);
      plotFunnel(c.V,c.lqr.x0);
      
      for i=1:5
        xtraj = simulate(sys,[0 6]);
        playback(pv,xtraj);
        figure(2);
        fnplt(xtraj);
      end
    end
  end
  
  properties
    lqr
    V
  end
  
end
