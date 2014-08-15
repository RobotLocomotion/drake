classdef COMState < SingletonCoordinateFrame
  methods
    function obj=COMState
      obj = obj@SingletonCoordinateFrame('COMState',6,'x');
    end
  end
end
