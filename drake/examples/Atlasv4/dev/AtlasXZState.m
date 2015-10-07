classdef AtlasXZState < SingletonCoordinateFrame
  
  methods
    function obj=AtlasXZState(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      manipStateFrame = r.getManipulator().getStateFrame();
      coordinates = manipStateFrame.getCoordinateNames;
      obj = obj@SingletonCoordinateFrame('AtlasXZState',length(coordinates),'x',coordinates);
    end
  end
end
