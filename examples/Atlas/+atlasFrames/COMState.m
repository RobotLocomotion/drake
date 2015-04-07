classdef COMState < SingletonCoordinateFrame
  methods
    function obj=COMState(dim)
      if nargin < 1
        dim = 3;
      end
      obj = obj@SingletonCoordinateFrame('atlasFrames.COMState',2*dim,'x');
    end
  end
end
