classdef CartPoleEnergyControl < HybridDrakeSystem
  
  methods
    function obj = CartPoleEnergyControl(plant)
      obj = obj@HybridDrakeSystem(4,1);
      
      obj = obj.addMode(CartPoleEnergyShaping(plant));
      [lqr,V] = balanceLQR(plant);
      V = V.inFrame(plant.getStateFrame);
      obj = obj.addMode(lqr);

      in_lqr_roa = @(t,~,x) V.eval(t,x)-1;
      notin_lqr_roa = notGuard(obj,in_lqr_roa);
      
      obj = obj.addTransition(1,in_lqr_roa,@transitionIntoLQR,true,true);
      obj = obj.addTransition(2,notin_lqr_roa,@transitionOutOfLQR,true,true);
    end

    function [xn,to_mode,status]=transitionIntoLQR(obj,mode,t,x,u)
      xn=[];
      to_mode=2
      status=0;
    end
    function [xn,to_mode,status]=transitionOutOfLQR(obj,mode,t,x,u)
      xn=[];
      to_mode=1
      status=0;
    end
    
  end
  
  methods (Static)
    function run()
      cp = CartPolePlant;
      cp = setInputLimits(cp,-inf,inf);  % need to do this for ROA code (for now)
      v = CartPoleVisualizer;
      c = CartPoleEnergyControl(cp);

      sys = feedback(cp,c);

      for i=1:4
        xtraj = simulate(sys,[0 10]);
        figure(2); clf; xtraj.fnplt([2;4]);
        playback(v,xtraj);
      end
    end
  end
    
end
