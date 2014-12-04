classdef AtlasXZState < SingletonCoordinateFrame
  
  methods
    function obj=AtlasXZState(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      manipStateFrame = r.getManipulator().getStateFrame();
      coordinates = manipStateFrame.coordinates;
      obj = obj@SingletonCoordinateFrame('AtlasXZState',length(coordinates),'x',coordinates);
    end
  end
end
