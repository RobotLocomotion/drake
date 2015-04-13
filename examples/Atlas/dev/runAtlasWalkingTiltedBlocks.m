function runAtlasWalkingTiltedBlocks()
%NOTEST
% Experimental test case for walking up stairs. Not yet reliable.

checkDependency('iris');
checkDependency('lcmgl');
path_handle = addpathTemporary(fullfile(getDrakePath(), 'examples', 'Atlas'));

robot_options = struct();
robot_options = applyDefaults(robot_options, struct('use_bullet', true,...
                                                    'terrain', RigidBodyFlatTerrain,...
                                                    'floating', true,...
                                                    'ignore_self_collisions', true,...
                                                    'ignore_effort_limits', true,...
                                                    'enable_fastqp', true,...
                                                    'ignore_friction', true,...
                                                    'use_new_kinsol', true,...
                                                    'hand_right', 'robotiq_weight_only',...
                                                    'hand_left', 'robotiq_weight_only',...
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

xstar(r.findPositionIndices('r_arm_usy')) = -0.931;
xstar(r.findPositionIndices('r_arm_shx')) = 0.717;
xstar(r.findPositionIndices('r_arm_ely')) = 1.332;
xstar(r.findPositionIndices('r_arm_elx')) = -0.871;
xstar(r.findPositionIndices('l_arm_usy')) = -0.931;
xstar(r.findPositionIndices('l_arm_shx')) = -0.717;
xstar(r.findPositionIndices('l_arm_ely')) = 1.332;
xstar(r.findPositionIndices('l_arm_elx')) = 0.871;
xstar = r.resolveConstraints(xstar);

r = r.setInitialState(xstar);
x0 = xstar;
nq = r.getNumPositions();

box_size = [0.39, 0.39, 0.146];

box_tops = [0.4, 0, 0.2, 0;
            0.4+0.4, 0, 0.2, pi/2;
            0.4+0.4*2, 0, 0.2, pi;
            0.4+0.4*3, 0, 0.2, -pi/2;
            0.4, -0.4, 0.2, -pi/2;
            0.4+0.4, -0.4, 0.2, 0;
            0.4+0.4*2, -0.4, 0.2, pi/2;
            0.4+0.4*3, -0.4, 0.35, pi]';

safe_regions = iris.TerrainRegion.empty();
[A, b] = poly2lincon([-1, 0.05, 0.05, -1], [-1, -1, 1, 1]);
[A, b] = convert_to_cspace(A, b);
safe_regions(1) = iris.TerrainRegion(A, b, [], [], [0;0;0], [0;0;1]);

for j = 1:size(box_tops, 2)
  rpy = [0;-15*pi/180;box_tops(4,j)];
  b = RigidBodyBox(box_size, box_tops(1:3,j) + [0;0;-box_size(3)/2], rpy);
  r = r.addGeometryToBody('world', b);
  offset = rpy2rotmat(rpy) * [0;0;box_size(3)/2];
  [A, b] = poly2lincon(box_tops(1,j) + offset(1) + [-0.03, 0.03, 0.03, -0.03],...
                       box_tops(2,j) + offset(2) + [-0.12, -0.12, 0.12, 0.12]);
  [A, b] = convert_to_cspace(A, b);
  normal = rpy2rotmat(rpy) * [0;0;1];
  safe_regions(end+1) = iris.TerrainRegion(A, b, [], [], box_tops(1:3,j), normal);
end
r = r.compile();
height_map = RigidBodyHeightMapTerrain.constructHeightMapFromRaycast(r,x0(1:nq),-1:.015:3,-1:.015:1,10);
r = r.setTerrain(height_map).compile();

v = r.constructVisualizer();
v.display_dt = 0.01;

footstep_plan = r.planFootsteps(x0(1:nq), struct('right',[1.65;-0.13;0;0;0;0],...
                                                 'left', [1.65;0.13;0;0;0;0]),...
                                safe_regions,...
                                struct('step_params', struct('max_forward_step', 0.4,...
                                                             'nom_forward_step', 0.05,...
                                                             'nom_upward_step', 0.3,...
                                                             'max_num_steps', 10)));
lcmgl = LCMGLClient('footsteps');
footstep_plan.draw_lcmgl(lcmgl);
lcmgl.switchBuffers();

% Snap to terrain
nsteps = length(footstep_plan.footsteps);
for j = 3:nsteps
  if ~footstep_plan.footsteps(j).pos_fixed(3)
    footstep_plan.footsteps(j) = fitStepToTerrain(r, footstep_plan.footsteps(j));
  end
end


% Add terrain profiles
for j = 3:length(footstep_plan.footsteps)
  [~, contact_width] = contactVolume(r, ...
                                        footstep_plan.footsteps(j-2), ...
                                        footstep_plan.footsteps(j));
  footstep_plan.footsteps(j).terrain_pts = sampleSwingTerrain(r, footstep_plan.footsteps(j-2), footstep_plan.footsteps(j), contact_width/2, struct());
end

walking_plan = r.planWalkingZMP(x0(1:nq), footstep_plan);

% ytraj = r.planWalkingStateTraj(walking_plan);
% v.playback(ytraj, struct('slider', true));
% keyboard()

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
T = walking_plan.duration;
ytraj = simulate(sys, [0, T], x0, struct('gui_control_interface', true));
[com, rms_com] = atlasUtil.plotWalkingTraj(r, ytraj, walking_plan);

v.playback(ytraj, struct('slider', true));


end

function [A, b] = convert_to_cspace(A, b)
  A = [A, zeros(size(A, 1), 1);
       zeros(2, size(A, 2) + 1)];
  A(end-1,3) = 1;
  A(end,3) = -1;
  b = [b; 0; 0];
end