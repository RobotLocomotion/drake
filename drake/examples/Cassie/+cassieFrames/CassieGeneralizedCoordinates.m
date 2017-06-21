classdef CassieGeneralizedCoordinates < SingletonCoordinateFrame
  methods
    function obj=CassieGeneralizedCoordinates(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      nq = r.getNumPositions();
      coords = r.getStateFrame().getCoordinateNames();
      obj = obj@SingletonCoordinateFrame('cassieFrames.CassieGeneralizedCoordinates',nq,'x',coords(1:nq));
    end
  end
end
