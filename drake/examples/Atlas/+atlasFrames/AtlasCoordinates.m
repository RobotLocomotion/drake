classdef AtlasCoordinates < SingletonCoordinateFrame
  % atlas q
  methods
    function obj=AtlasCoordinates(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      nq = r.getNumPositions();
      coords = r.getStateFrame().getCoordinateNames();
      obj = obj@SingletonCoordinateFrame('atlasFrames.AtlasCoordinates',nq,'x',coords(1:nq));
    end
  end
end
