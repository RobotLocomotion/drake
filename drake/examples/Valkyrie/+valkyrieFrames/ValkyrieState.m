classdef ValkyrieState < SingletonCoordinateFrame

  methods
    function obj=ValkyrieState(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      manipStateFrame = r.getManipulator().getStateFrame();

      coordinates = manipStateFrame.getCoordinateNames();
      obj = obj@SingletonCoordinateFrame('valkyrieFrames.ValkyrieState',length(coordinates),'x',coordinates);
      positionFrame = r.getManipulator().getPositionFrame();
      if getNumFrames(positionFrame)==1 && isempty(findTransform(obj,positionFrame))
        obj.addProjectionTransformByCoordinateNames(positionFrame);
      end
    end
  end
end
