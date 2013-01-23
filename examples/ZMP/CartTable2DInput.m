function fr = CartTable2DInput

r = PlanarRigidBodyManipulator('CartTable.urdf',struct('floating',true,'view','right'));
fr = r.getInputFrame();
