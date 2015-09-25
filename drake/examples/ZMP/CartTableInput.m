function fr = CartTableInput

path_to_this_file = fileparts(which(mfilename));
r = RigidBodyManipulator(fullfile(path_to_this_file,'CartTable.urdf'),struct('floating',true));
fr = r.getInputFrame();

% NOTEST

