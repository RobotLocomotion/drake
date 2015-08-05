classdef OneLegHopperState < SingletonCoordinateFrame
  methods
    function obj=OneLegHopperState(r)
      manipStateFrame = r.getManipulator().getStateFrame();
      coordinates = manipStateFrame.coordinates;
      obj = obj@SingletonCoordinateFrame('OneLegHopperState',length(coordinates),'x',coordinates);
    end
  end
end
