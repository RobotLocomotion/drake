function runAtlasBalancing(sim_options)
% put robot in a random x,y,yaw position and balance for 2 seconds

if nargin < 1
  sim_options = struct();
end

checkDependency('gurobi')

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

options.floating = true;
options.dt = 0.002;
r = Atlas('urdf/atlas_minimal_contact.urdf',options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

r.runBalancingDemo(sim_options);
