classdef PendulumInput < LCMCoordinateFrame & Singleton
  
  methods
    function obj=PendulumInput()
      obj = obj@LCMCoordinateFrame('PendulumInput','drake.examples.Pendulum.lcmt_pendulum_u','u',{'torque'});
      obj = obj@Singleton();
    end
  end
end
