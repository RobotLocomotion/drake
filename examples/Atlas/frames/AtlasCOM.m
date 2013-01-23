classdef AtlasCOM < SingletonCoordinateFrame
  % simple frame for Atlas COM
  methods
    function obj=AtlasCOM(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      obj = obj@SingletonCoordinateFrame('COMGoal',3,'x');
    end
  end
end
