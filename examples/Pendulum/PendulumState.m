classdef PendulumState < LCMCoordinateFrame & Singleton
  
  methods
    function obj=PendulumState()
      obj = obj@LCMCoordinateFrame('PendulumState','drake.examples.Pendulum.lcmt_pendulum_x','x',{'theta','thetadot'});
      obj = obj@Singleton();
    end
  end
end
