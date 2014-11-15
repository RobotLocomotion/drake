classdef AtlasState < SingletonCoordinateFrame
  
  methods
    function obj=AtlasState(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      manipStateFrame = r.getManipulator().getStateFrame();
      % sanity check for stateless hands
      if (r.hands > 0)
        manipStateFrame = manipStateFrame.getFrameByNum(1);
      end
      coordinates = manipStateFrame.coordinates;
      obj = obj@SingletonCoordinateFrame('AtlasState',length(coordinates),'x',coordinates);
    end
  end
end
