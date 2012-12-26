classdef PendulumState < LCMCoordinateFrame
  
  methods
    function obj=PendulumState()
      obj = obj@LCMCoordinateFrame('PendulumState','drake.examples.Pendulum.lcmt_pendulum_x','x');
%      obj.setCoordinateNames({'\theta','\dot\theta'});
    end
  end
end
