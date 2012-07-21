classdef PendulumInput < LCMCoordinateFrame
  
  methods
    function obj=PendulumInput()
      obj = obj@LCMCoordinateFrame('PendulumInput','drake.examples.Pendulum.lcmt_pendulum_u','u');
%      obj.setCoordinateNames({'\tau'});
    end
  end
end
