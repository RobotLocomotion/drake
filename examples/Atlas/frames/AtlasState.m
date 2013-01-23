classdef AtlasState < SingletonCoordinateFrame
  
  methods
    function obj=AtlasState(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');

      obj = obj@SingletonCoordinateFrame('AtlasState',r.getNumStates(),'x',r.getStateFrame.coordinates);
    end
  end
end
