classdef COMAcceleration < SingletonCoordinateFrame
  methods
    function obj=COMAcceleration(dim)
      if nargin < 1
        dim = 3;
      end
      obj = obj@SingletonCoordinateFrame('valkyrieFrames.COMAcceleration',dim,'c');
    end
  end
end
