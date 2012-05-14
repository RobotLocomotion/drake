classdef PendulumInput < CoordinateFrame & Singleton
  
  methods
    function obj=PendulumInput()
      obj = obj@CoordinateFrame('PendulumInput',1,'u');
      obj.setCoordinateNames({'\tau'});
    end
  end
end
