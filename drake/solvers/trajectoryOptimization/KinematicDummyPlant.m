classdef KinematicDummyPlant < LinearSystem
  properties (SetAccess = protected)
    robot
  end

  methods
    function obj = KinematicDummyPlant(robot)
      nq = robot.getNumPositions();

      A = [zeros(nq), eye(nq); zeros(nq,2*nq)];
      B = [zeros(nq);eye(nq)];

      obj  = obj@LinearSystem(A,B,[],[],[],[]);
      obj.robot = robot;
    end
  end
end
