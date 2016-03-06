function fr = CartTable2DInput

S = warning('off','Drake:PlanarRigidBodyManipulator:RemovedJoint');
path_to_this_file = fileparts(which(mfilename));
r = PlanarRigidBodyManipulator(fullfile(path_to_this_file,'CartTable.urdf'),struct('floating',true,'view','right'));
fr = r.getInputFrame();
warning(S);

% NOTEST

