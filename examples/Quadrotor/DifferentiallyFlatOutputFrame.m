classdef DifferentiallyFlatOutputFrame < SingletonCoordinateFrame

  methods
    function obj=DifferentiallyFlatOutputFrame()
      obj = obj@SingletonCoordinateFrame('QuadrotorDifferentiallyFlatOutput',4,'y',{'x','y','z','yaw'});
    end
  end  
end