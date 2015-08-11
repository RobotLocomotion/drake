function runAtlasWalkingNarrowStairs()
% Climb a set of stairs which are intentionally shorter than the robot's feet. This forces the controller to explicitly plan for and control only partial foot contact. 

checkDependency('iris');
checkDependency('lcmgl');
path_handle = addpathTemporary(fullfile(getDrakePath(), 'examples', 'Atlas'));

robot_options = struct();
robot_options = applyDefaults(robot_options, struct('use_bullet', false,...
                                                    'terrain', RigidBodyFlatTerrain(-0.44),...
                                                    'floating', true,...
                                                    'ignore_self_collisions', true,...
                                                    'enable_fastqp', false,...
                                                    'ignore_friction', true,...
                                                    'hand_right', 'robotiq_weight_only',...
                                                    'hand_left', 'robotiq_weight_only',...
                                                    'dt', 0.001));
% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

% construct robot model
r = Atlas(fullfile(getDrakePath,'examples','Atlas','urdf','atlas_minimal_contact.urdf'),robot_options);
r = r.removeCollisionGroupsExcept({'heel','toe','midfoot_front', 'midfoot_rear'});
r = compile(r);

% set initial state to fixed point
load(fullfile(getDrakePath,'examples','Atlas','data','atlas_fp.mat'));
% Move the robot's arms out in front to bring the CoM forward
xstar(6) = pi/2;
xstar(r.findPositionIndices('r_arm_shz')) = 1.3;
xstar(r.findPositionIndices('r_arm_shx')) = 0;
xstar(r.findPositionIndices('r_arm_elx')) = -0.5;
xstar(r.findPositionIndices('r_arm_ely')) = 0;
xstar(r.findPositionIndices('l_arm_shz')) = -1.3;
xstar(r.findPositionIndices('l_arm_shx')) = 0;
xstar(r.findPositionIndices('l_arm_elx')) = 0.5;
xstar(r.findPositionIndices('l_arm_ely')) = 0;
xstar = r.resolveConstraints(xstar);

r = r.setInitialState(xstar);
x0 = xstar;
nq = r.getNumPositions();

% Generate boxes and IRIS safe terrain regions
l = 0.30;
h = 0.15;
box_size = [39*0.0254, l, h];

box_tops = [0, 0.2-l, 0;
            0, 0.2, 0;
            0, 0.2+l, -h;
            0, 0.2+2*l, -h*2]';

safe_regions = iris.TerrainRegion.empty();

for j = 1:size(box_tops, 2)
  b = RigidBodyBox(box_size, box_tops(:,j) + [0;0;-box_size(3)/2], [0;0;0]);
  r = r.addGeometryToBody('world', b);
  [A, b] = poly2lincon(box_tops(1,j) + [-0.25, -0.25, 0.25, 0.25],...
                       box_tops(2,j) + [0.01, 0.0, 0.0, 0.01]);
  [A, b] = convertToCspace(A, b);
  safe_regions(end+1) = iris.TerrainRegion(A, b, [], [], box_tops(1:3,j), [0;0;1]);
end
r = r.compile();
x0_scan = x0;
x0_scan(1) = x0_scan(1) - 10; % move the robot away so we don't scan it into the terrain
height_map = RigidBodyHeightMapTerrain.constructHeightMapFromRaycast(r,x0_scan(1:nq),-1:.015:1,-1:.015:1.5,10);
r = r.setTerrain(height_map).compile();

v = r.constructVisualizer();
v.display_dt = 0.01;

% Plan footsteps
footstep_plan = r.planFootsteps(x0(1:nq), struct('right',[0.13;.75;0;0;0;0],...
                                                 'left', [-0.13;.75;0;0;0;0]),...
                                safe_regions,...
                                struct('step_params', struct('max_forward_step', 0.4,...
                                                             'nom_forward_step', 0.025,...
                                                             'max_num_steps', 8)));
lcmgl = LCMGLClient('footsteps');
footstep_plan.draw_lcmgl(lcmgl);
lcmgl.switchBuffers();

% Add terrain profiles
for j = 3:length(footstep_plan.footsteps)
  [~, contact_width] = contactVolume(r, ...
                                        footstep_plan.footsteps(j-2), ...
                                        footstep_plan.footsteps(j));
  footstep_plan.footsteps(j).terrain_pts = sampleSwingTerrain(r, footstep_plan.footsteps(j-2), footstep_plan.footsteps(j), contact_width/2, struct());
end

lcmgl = LCMGLClient('walking plan');
walking_plan = r.planWalkingZMP(x0(1:nq), footstep_plan);
walking_plan.settings.draw_lcmgl(lcmgl);
lcmgl.switchBuffers();

% ytraj = r.planWalkingStateTraj(walking_plan.settings, xstar);
% v.playback(ytraj, struct('slider', true));
% keyboard()

[ytraj, com, rms_com] = atlasUtil.simulateWalking(r, walking_plan);

v.playback(ytraj, struct('slider', true));

if ~rangecheck(rms_com, 0, 0.01);
  error('Drake:runAtlasWalkingStairs:BadCoMTracking', 'Center-of-mass during execution differs substantially from the plan.');
end

end

function [A, b] = convertToCspace(A, b)
  A = [A, zeros(size(A, 1), 1);
       zeros(2, size(A, 2) + 1)];
  A(end-1,3) = 1;
  A(end,3) = -1;
  b = [b; pi/2; -pi/2];
end

% TIMEOUT 1500