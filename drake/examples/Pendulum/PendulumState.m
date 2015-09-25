classdef PendulumState < LCMCoordinateFrame & Singleton
  
  methods
    function obj=PendulumState()
      obj = obj@LCMCoordinateFrame('PendulumState','lcmdrake.lcmt_drake_signal','x',{'theta','thetadot'});
      obj = obj@Singleton();
    end
  end
end
