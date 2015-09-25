function fr = CartTable2DState
  % note: the actual frame name is CartTableXZState (and it's not worth
  % changing it)

S = warning('off','Drake:PlanarRigidBodyManipulator:RemovedJoint');
path_to_this_file = fileparts(which(mfilename));
r = PlanarRigidBodyManipulator(fullfile(path_to_this_file,'CartTable.urdf'),struct('floating',true,'view','right'));
fr = r.getStateFrame();
warning(S);

% NOTEST
