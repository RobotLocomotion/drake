function fr = CartTableInput

r = RigidBodyManipulator(fullfile(getDrakePath, '+examples', '+ZMP', 'CartTable.urdf'),struct('floating',true));
fr = r.getInputFrame();

% NOTEST

