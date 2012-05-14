classdef PendulumState < CoordinateFrame & Singleton
  
  methods
    function obj=PendulumState()
      obj = obj@CoordinateFrame('PendulumState',2,'x');
      obj.setCoordinateNames({'\theta','\dot\theta'});
      obj.setAngleFlags([true;false]);
    end
  end
end
