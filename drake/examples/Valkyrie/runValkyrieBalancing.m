function runValkyrieBalancing(sim_options)
% put robot in a random x,y,yaw position and balance for 2 seconds

checkDependency('gurobi')

if (nargin<1); sim_options = struct(); end

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

options.floating = true;
options.dt = 0.002;

r = Valkyrie(fullfile(getDrakePath,'examples','Valkyrie','urdf','urdf','valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf'),options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

r.runBalancingDemo(sim_options);
