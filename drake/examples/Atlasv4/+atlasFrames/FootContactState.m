classdef FootContactState < CoordinateFrame & Singleton
  methods
    function obj=FootContactState()
      obj = obj@CoordinateFrame('atlasFrames.FootContactState',2,'x',{'left','right'});
      obj = obj@Singleton();
    end
  end
end
