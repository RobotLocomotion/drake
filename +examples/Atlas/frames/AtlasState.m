classdef AtlasState < SingletonCoordinateFrame
  
  methods
    function obj=AtlasState(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');

      obj = obj@SingletonCoordinateFrame('AtlasState',r.getManipulator().getNumStates(),'x',r.getManipulator().getStateFrame.coordinates);
    end
  end
end
