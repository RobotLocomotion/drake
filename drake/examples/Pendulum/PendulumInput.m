classdef PendulumInput < LCMCoordinateFrame & Singleton
  
  methods
    function obj=PendulumInput()
      obj = obj@LCMCoordinateFrame('PendulumInput','drake.lcmt_drake_signal','u',{'torque'});
      obj = obj@Singleton();
    end
  end
end
