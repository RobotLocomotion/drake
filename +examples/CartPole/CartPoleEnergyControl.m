classdef CartPoleEnergyControl < HybridDrakeSystem
  
  methods
    function obj = CartPoleEnergyControl(plant)
      obj = obj@HybridDrakeSystem(4,1);
      obj = obj.setInputFrame(plant.getStateFrame);
      obj = obj.setOutputFrame(plant.getInputFrame);
      
      obj = obj.addMode(CartPoleEnergyShaping(plant));
      [lqr,V] = balanceLQR(plant);
      obj.V = V.inFrame(plant.getStateFrame);
      obj = obj.addMode(lqr);

      in_lqr_roa = @(obj,t,~,x) obj.V.eval(t,x)-1;
      notin_lqr_roa = @(obj,t,~,x) 1-obj.V.eval(t,x); 
      
      obj = obj.addTransition(1,in_lqr_roa,@transitionIntoLQR,true,true,2);
      obj = obj.addTransition(2,notin_lqr_roa,@transitionOutOfLQR,true,true,1);
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
      v = CartPoleVisualizer(cp);
      c = CartPoleEnergyControl(cp);

      sys = feedback(cp,c);

      for i=1:4
        [ytraj,xtraj] = simulate(sys,[0 10]);
        figure(2); clf; ytraj.fnplt([2;4]);
        playback(v,ytraj);
        if (isa(xtraj,'HybridTrajectory') && length(xtraj.getEvents())>1)
          m = cellfun(@(a) subsref(a.eval(a.tspan(1)),substruct('()',{1})), xtraj.traj);
          keyboard
          if (any(find(m==1)>find(m==2,1,'first')))
            error('The controller transitioned out of the region verified as invariant for the balancing controller');
          end
        end
      end
    end
  end
  
  properties
    V
  end
    
end
