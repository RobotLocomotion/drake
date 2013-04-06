classdef AtlasCoordinates < SingletonCoordinateFrame
  % atlas q
  methods
    function obj=AtlasCoordinates(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      nq = r.getNumDOF();
      obj = obj@SingletonCoordinateFrame('AtlasCoordinates',nq,'x',r.getStateFrame.coordinates(1:nq));
    end
  end
end
