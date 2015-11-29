classdef PendulumEnergyControl < HybridDrakeSystem
  
  methods
    function obj = PendulumEnergyControl(plant)
      typecheck(plant,'PendulumPlant');
      obj = obj@HybridDrakeSystem(2,1);
      obj = setInputFrame(obj,PendulumState);
      obj = setOutputFrame(obj,PendulumInput);
      obj = obj.addMode(PendulumEnergyShaping(plant));

      [lqr,V] = balanceLQR(plant);
      
      % enable wrapping
      lqr = setInputFrame(lqr,lqr.getInputFrame.constructFrameWithAnglesWrapped([1;0]));
      V = setFrame(V,lqr.getInputFrame);
      
      obj.V = V.inFrame(plant.getStateFrame);
      obj = obj.addMode(lqr);

      in_lqr_roa = @(obj,t,~,x) obj.V.eval(t,x)-1;
      notin_lqr_roa = @(obj,t,~,x) 1-obj.V.eval(t,x); 
      
      obj = obj.addTransition(1,in_lqr_roa,@transitionToLQR,true,true,2);
      obj = obj.addTransition(2,notin_lqr_roa,@transitionFromLQR,true,true,1);

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
      pd = setInputLimits(pd,-inf,inf);
      pv = PendulumVisualizer();
      c = PendulumEnergyControl(pd);

      sys = feedback(pd,c);
      
      figure(2); clf; hold on;
      options.x0 = [pi;0];
      plotFunnel(c.V.inFrame(pd.getStateFrame),options);
%      options.x0 = [-pi;0];
%      plotFunnel(c.V.inFrame(pd.getStateFrame),options);
      
      for i=1:5
        xtraj = simulate(sys,[0 6]);
        playback(pv,xtraj);
        figure(2); fnplt(xtraj);
      end
    end
  end
  
  properties
    V
  end
  
end
