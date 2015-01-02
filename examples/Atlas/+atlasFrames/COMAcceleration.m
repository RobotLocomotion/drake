classdef COMAcceleration < SingletonCoordinateFrame
  methods
    function obj=COMAcceleration
      obj = obj@SingletonCoordinateFrame('atlasFrames.COMAcceleration',3,'c');
    end
  end
end
