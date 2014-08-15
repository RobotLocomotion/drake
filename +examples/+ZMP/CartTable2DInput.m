function fr = CartTable2DInput

S = warning('off','Drake:PlanarRigidBodyManipulator:RemovedJoint');
r = PlanarRigidBodyManipulator('CartTable.urdf',struct('floating',true,'view','right'));
fr = r.getInputFrame();
warning(S);

% NOTEST

