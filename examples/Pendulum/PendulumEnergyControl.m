classdef PendulumEnergyControl < FiniteStateMachine
  
  methods
    function obj = PendulumEnergyControl(plant)
      obj = obj.addMode(PendulumEnergyShaping(plant));
      lqr = PendulumLQR(plant);
      obj = obj.addMode(lqr);

      % todo: build closed-loop lqr system and do verification on it to
      % determine transition boundary.
      obj.x0 = lqr.x0;
      obj.S = lqr.S;
      obj.rho = 4;

      lqr_in = inline('(x-obj.x0)''*obj.S*(x-obj.x0) - obj.rho','obj','t','junk','x');
      lqr_out = inline('obj.rho - (x-obj.x0)''*obj.S*(x-obj.x0)','obj','t','junk','x');
      
      obj = obj.addTransition(1,2,lqr_in,[],true,true);
      obj = obj.addTransition(2,1,lqr_out,[],true,true);
      
      obj.output_mode = false;
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