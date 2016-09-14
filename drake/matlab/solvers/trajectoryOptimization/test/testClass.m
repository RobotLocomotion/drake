% NOTEST

classdef testClass < KinematicTrajectoryOptimization & NonlinearProgram
  
  properties
    N;
    Xinds;
  end
  
  methods
    
    function obj = testClass(robot)
      obj = obj@NonlinearProgram(0);
      obj = obj@KinematicTrajectoryOptimization(robot,0);
      
      obj.N = 1;
      nq = robot.getNumPositions();
      obj.Xinds = [1:nq]';
      obj = obj.addDecisionVariable(nq);
      obj = obj.initialize();
    end
    
    % required method of KinematicTrajectoryOptimization
    function Xinds = getXinds(obj)
      Xinds = obj.Xinds;
    end
    
    % required method of KinematicTrajectoryOptimization
    function Hinds = getHinds(obj)
      Hinds = [];
    end
    
    % required abstract method of KinematicTrajectoryOptimization
    function N = getN(obj)
      N = obj.N;
    end
    
  end
end