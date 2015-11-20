classdef ValkyrieCoordinates < SingletonCoordinateFrame
  % valkyrie q
  methods
    function obj=ValkyrieCoordinates(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      nq = r.getNumPositions();
      coords = r.getStateFrame().getCoordinateNames();
      obj = obj@SingletonCoordinateFrame('valkyrieFrames.ValkyrieCoordinates',nq,'x',coords(1:nq));
    end
  end
end
