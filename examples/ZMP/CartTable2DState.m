function fr = CartTable2DState
  % note: the actual frame name is CartTableXZState (and it's not worth
  % changing it)
r = PlanarRigidBodyManipulator('CartTable.urdf',struct('floating',true,'view','right'));
fr = r.getStateFrame();

