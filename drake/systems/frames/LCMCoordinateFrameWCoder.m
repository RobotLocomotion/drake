classdef LCMCoordinateFrameWCoder < LCMCoordinateFrame
  
  methods
    function obj = LCMCoordinateFrameWCoder(name,dim,prefix,lcmcoder)
      warning('Drake:Deprecated','The LCMCoordinateFrameWCoder interface is deprecated.  Please use LCMCoordinateFrame directly (it takes a coder)');
      obj = obj@LCMCoordinateFrame(name,lcmcoder,prefix);
    end
  end
  
end