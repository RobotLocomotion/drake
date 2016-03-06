function runAtlasDynamics
% Simulate the (passive) dynamics of the Atlas model 

% Load the model with a floating base
options.floating = true;
options.dt = 0.001;
options.terrain = RigidBodyFlatTerrain;
r = Atlas('urdf/atlas_convex_hull.urdf',options);
%r = r.removeCollisionGroupsExcept({'heel','toe','back','front','knee','butt'});
r = compile(r);

% Initialize the viewer
v = r.constructVisualizer;
v.display_dt = 0.02;

% Compute a feasible set of initial conditions for the simulation (e.g. no
% penetration)
x0 = Point(r.getStateFrame);
x0 = resolveConstraints(r,x0);

% Forward simulate dynamics with visulazation, then playback at realtime
S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(r,v,[],[],output_select);
warning(S);
traj = simulate(sys,[0 2],x0);
playback(v,traj,struct('slider',true));

end
