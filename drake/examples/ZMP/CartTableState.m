function fr = CartTableState

r = RigidBodyManipulator('CartTable.urdf',struct('floating',true));
fr = r.getStateFrame();

% NOTEST

