classdef CartPoleEnergyControl < FiniteStateMachine
  
  methods
    function obj = CartPoleEnergyControl(plant)
      obj = obj.addMode(CartPoleEnergyShaping(plant));
      lqr = CartPoleLQR(plant);
      obj = obj.addMode(lqr);

      % todo: build closed-loop lqr system and do verification on it to
      % determine transition boundary.
%      obj.x0 = [pi;0];
      obj.x0 = lqr.x0;
%      obj.S = lqr.S([2,4],[2,4]);
      obj.S = lqr.S;
      obj.rho = 100;

      in_lqr_roa = inline('(x-obj.x0)''*obj.S*(x-obj.x0) - obj.rho','obj','t','junk','x');
%      in_lqr_roa = inline('(x([2,4])-obj.x0)''*obj.S*(x([2,4])-obj.x0) - obj.rho','obj','t','junk','x');
      notin_lqr_roa = not_guard(obj,in_lqr_roa);
      
      obj = obj.addTransition(1,2,in_lqr_roa,[],true,true);
      obj = obj.addTransition(2,1,notin_lqr_roa,[],true,true);
      
      obj.output_mode = false;
    end
    
  end
  
  methods (Static)
    function run()
      cp = CartPolePlant;
      v = CartPoleVisualizer;
      c = CartPoleEnergyControl(cp);

      sys = feedback(cp,c);

      for i=1:5
        xtraj = simulate(sys,[0 10]);
        playback(v,xtraj);
      end
    end
  end
  
  properties
    x0
    S
    rho
  end
  
end