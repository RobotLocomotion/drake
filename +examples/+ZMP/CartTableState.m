function fr = CartTableState

r = RigidBodyManipulator(fullfile(getDrakePath, '+examples', '+ZMP', 'CartTable.urdf'),struct('floating',true));
fr = r.getStateFrame();

% NOTEST

