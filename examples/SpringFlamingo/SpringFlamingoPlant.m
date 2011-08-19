classdef SpringFlamingoPlant < HybridSimulationConstructionSetRobot
  
  methods
    function obj=SpringFlamingoPlant(r)
      obj = obj@HybridSimulationConstructionSetRobot(r);
      typecheck(r,'com.yobotics.exampleSimulations.springflamingo.SpringFlamingoRobot');
%      obj = setSimulinkParam(obj,'Solver','ode4','FixedStep','.0001');
      obj = setSimulinkParam(obj,'MinStep','0.0001');
    end
    
    function x0 = getInitialState(obj)
      r=obj.robotobj;
      r.initializeForBallisticWalking();
      x0 = r.getStateVector(); 
%      x0(16+2)=x0(16+2)+.2;

      m = modeNumFromGCboolean(obj,zeroCrossings(obj,0,[1;x0],zeros(6,1))<0);
      x0=[m;obj.robotobj.matlabGroundContactTransition(x0)];
    end
  end

  methods (Static=true)
    function run()
      p=SpringFlamingoPlant.build();
      pv = SpringFlamingoVisualizer();
      traj = simulate(p,[0 .45]);
      pv.playback_speed = .05;
      playback(pv,traj);
    end
    
    function p=build()
      eclipse_root = '/Users/russt/Documents/workspace/';
      javaaddpath([eclipse_root,'SpringFlamingo/classes']);

      r = com.yobotics.exampleSimulations.springflamingo.SpringFlamingoRobot('SpringFlamingo');
      r.setGroundContactModel(com.yobotics.simulationconstructionset.util.LinearGroundContactModel(r, 14220, 150.6, 125.0, 300.0,[]));
      p=SpringFlamingoPlant(r);
    end
    
    function simulateWJerrysControl()
      % Simulates Spring Flamingo with Jerry's State Machine Controller attached, and with the initial conditions from Jerry's simulation
      %
      % NOTE: you have to comment out the setInputVector command in 
      % Robot.matlabDynamics for this to work again.  (see bug 835)
      
      p=SpringFlamingoPlant.build();
      p.robotobj.setController(com.yobotics.exampleSimulations.springflamingo.SpringFlamingoController(p.robotobj, 'springFlamingoController'), 4);
      p = setSimulinkParam(p,'Solver','ode4','FixedStep','.0001');
      pv = SpringFlamingoVisualizer();
     
      if (1)
        sys=cascade(sampledData(p,0,.01),pv);
        simulate(sys,[0 inf]);
      else
        traj=simulate(p,[0 10]);
        save jerry.mat traj;
        playback(pv,traj);
%        playbackSWF(pv,traj,'jerry.swf');
      end
    end
  end
  
end
