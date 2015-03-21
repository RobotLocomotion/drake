classdef AtlasState < SingletonCoordinateFrame
  
  methods
    function obj=AtlasState(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      manipStateFrame = r.getManipulator().getStateFrame();
      % sanity check for stateless hands
      if (r.hand_left > 0 || r.hand_right)
        manipStateFrame = manipStateFrame.getFrameByNum(1);
      end
      coordinates = manipStateFrame.coordinates;
      obj = obj@SingletonCoordinateFrame('atlasFrames.AtlasState',length(coordinates),'x',coordinates);
    end
  end
end
