function fr = CartTableInput

r = RigidBodyManipulator('CartTable.urdf',struct('floating',true));
fr = r.getInputFrame();

% NOTEST

