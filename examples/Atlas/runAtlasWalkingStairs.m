function runAtlasWalkingStairs()
checkDependency('iris');
checkDependency('lcmgl');

robot_options = struct();
robot_options = applyDefaults(robot_options, struct('use_bullet', false,...
                                                    'terrain', RigidBodyFlatTerrain,...
                                                    'floating', true,...
                                                    'ignore_self_collisions', true,...
                                                    'ignore_friction', true,...
                                                    'dt', 0.001));
% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

% construct robot model
r = Atlas(fullfile(getDrakePath,'examples','Atlas','urdf','atlas_minimal_contact.urdf'),robot_options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

% set initial state to fixed point
load(fullfile(getDrakePath,'examples','Atlas','data','atlas_fp.mat'));
xstar = r.resolveConstraints(xstar);
r = r.setInitialState(xstar);
x0 = xstar;
nq = r.getNumPositions();

box_size = [0.3, 0.75, 0.15];

box_tops = [0.5, 0, 0.15;
            0.8, 0, 0.3]';

lcmgl = LCMGLClient('iris_regions');
[A, b] = poly2lincon([-1, 0.18, 0.18, -1],...
                     [-1, -1, 1, 1]);
[A, b] = convert_to_cspace(A, b);
safe_regions = iris.TerrainRegion(A, b, [], [], [0;0;0], [0;0;1]);

for j = 1:size(box_tops, 2)
  b = RigidBodyBox(box_size, box_tops(:,j) + [0;0;-box_size(3)/2], [0;0;0]);
  r = r.addGeometryToBody('world', b);
  [A, b] = poly2lincon(box_tops(1,j) + [-0.01, 0.01, 0.01, -0.01],...
                       box_tops(2,j) + [-1, -1, 1, 1]);
  [A, b] = convert_to_cspace(A, b);
  safe_regions(end+1) = iris.TerrainRegion(A, b, [], [], box_tops(1:3,j), [0;0;1]);
end
r = r.compile();
height_map = RigidBodyHeightMapTerrain.constructHeightMapFromRaycast(r,x0(1:nq),-1:.015:3,-1:.015:1,10);
r = r.setTerrain(height_map).compile();

v = r.constructVisualizer();

lcmgl.glColor3f(0,0,0);
for j = 1:length(safe_regions)
  safe_regions(j).getXYZPolytope().drawLCMGL(lcmgl);
end


footstep_plan = r.planFootsteps(x0(1:nq), struct('right',[3;-0.13;0;0;0;0],...
                                                 'left', [3;0.13;0;0;0;0]),...
                                safe_regions,...
                                struct('step_params', struct('max_forward_step', 0.4,...
                                                             'nom_foward_step', 0.1,...
                                                             'max_num_steps', 10)));
footstep_plan.draw_lcmgl(lcmgl);

% Add terrain profiles
for j = 3:length(footstep_plan.footsteps)
  [~, contact_width] = contactVolume(r, ...
                                        footstep_plan.footsteps(j-2), ...
                                        footstep_plan.footsteps(j));
  footstep_plan.footsteps(j).terrain_pts = sampleSwingTerrain(r, footstep_plan.footsteps(j-2), footstep_plan.footsteps(j), contact_width/2, struct());
end

walking_plan = r.planWalkingZMP(x0(1:nq), footstep_plan);

lcmgl.switchBuffers();

ytraj = r.planWalkingStateTraj(walking_plan);
v.playback(ytraj, struct('slider', true));

% Build our controller and plan eval objects
control = atlasControllers.InstantaneousQPController(r, []);
planeval = atlasControllers.AtlasPlanEval(r, walking_plan);
plancontroller = atlasControllers.AtlasPlanEvalAndControlSystem(r, control, planeval);
sys = feedback(r, plancontroller);

% Add a visualizer
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);

% Simulate and draw the result
T = min(walking_plan.duration + 1, 30);
ytraj = simulate(sys, [0, T], x0, struct('gui_control_interface', true));

v.playback(ytraj, struct('slider', true));


end

function [A, b] = convert_to_cspace(A, b)
  A = [A, zeros(size(A, 1), 1)];
       % zeros(2, size(A, 2) + 1)];
  % A(end-1,3) = 1;
  % A(end,3) = -1;
  % b = [b; pi/8; pi/8];
end