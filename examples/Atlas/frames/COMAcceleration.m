classdef COMAcceleration < SingletonCoordinateFrame
  methods
    function obj=COMAcceleration
      obj = obj@SingletonCoordinateFrame('COMAcceleration',3,'c');
    end
  end
end
