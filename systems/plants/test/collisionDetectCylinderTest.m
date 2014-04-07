function collisionDetectCylinderTest(varargin)
  %NOTEST - Bullet cylinders don't play nicely with numerical differentiation.
  collisionDetectTest('Cylinder.urdf',varargin{:});
end
