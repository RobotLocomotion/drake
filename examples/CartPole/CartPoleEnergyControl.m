classdef CartPoleEnergyControl < HybridRobotLibSystem
  
  methods
    function obj = CartPoleEnergyControl(plant)
      obj = obj.addMode(CartPoleEnergyShaping(plant));
      lqr = CartPoleLQR(plant);
      obj = obj.addMode(lqr);

      sys = feedback(plant,lqr);
      pp = sys.taylorApprox(0,lqr.x0,[],3);  % make polynomial approximation
      obj.x0 = lqr.x0;
      obj.p_x = pp.p_x;
      options.degL1 = 1;
      obj.V = regionOfAttraction(pp,lqr.x0,lqr.S,options);

      in_lqr_roa = inline('double(subs(obj.V,obj.p_x,obj.wrapInput(x-obj.x0)+obj.x0))-1','obj','t','junk','x');
      notin_lqr_roa = notGuard(obj,in_lqr_roa);
      
      obj = obj.addTransition(1,in_lqr_roa,@transitionIntoLQR,true,true);
      obj = obj.addTransition(2,notin_lqr_roa,@transitionOutOfLQR,true,true);
      
      obj = setModeOutputFlag(obj,false);
    end

    function [xn,to_mode,status]=transitionIntoLQR(obj,mode,t,x,u)
      xn=[];
      to_mode=2;
      status=0;
    end
    function [xn,to_mode,status]=transitionOutOfLQR(obj,mode,t,x,u)
      xn=[];
      to_mode=1;
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

      for i=1:2
        xtraj = simulate(sys,[0 10]);
        playback(v,xtraj);
      end
    end
  end
  
  properties
    x0
    p_x
    V
  end
  
end
