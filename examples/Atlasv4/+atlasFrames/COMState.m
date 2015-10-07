classdef COMState < SingletonCoordinateFrame
  methods
    function obj=COMState
      obj = obj@SingletonCoordinateFrame('atlasFrames.COMState',6,'x');
    end
  end
end
