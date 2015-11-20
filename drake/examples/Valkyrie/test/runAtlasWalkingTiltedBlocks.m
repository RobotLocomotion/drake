function runAtlasWalkingTiltedBlocks()
% Run atlas across the tilted cinderblock terrain from the DRC testbed

checkDependency('iris');
checkDependency('lcmgl');
path_handle = addpathTemporary(fullfile(getDrakePath(), 'examples', 'Atlas'));

robot_options = struct();
robot_options = applyDefaults(robot_options, struct('use_bullet', true,...
                                                    'terrain', RigidBodyFlatTerrain,...
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
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

% set initial state to fixed point
load(fullfile(getDrakePath,'examples','Atlas','data','atlas_fp.mat'));
xstar = r.resolveConstraints(xstar);

r = r.setInitialState(xstar);
x0 = xstar;
nq = r.getNumPositions();

box_size = [0.39, 0.39, 0.146];

box_tops = [0.4, 0, 0.2, 0;
            0.4+0.4, 0, 0.2, pi/2;
            0.4+0.4*2, 0, 0.2, pi;
            0.4+0.4*3, 0, 0.2, -pi/2;
            0.4+0.4*4, 0, 0.35, 0;
            0.4+0.4*5, 0, 0.35, pi/2;
            0.4+0.4*6, 0, 0.2, pi;
            0.4, -0.4, 0.2, -pi/2;
            0.4+0.4, -0.4, 0.2, 0;
            0.4+0.4*2, -0.4, 0.2, pi/2;
            0.4+0.4*3, -0.4, 0.35, pi;
            0.4+0.4*4, -0.4, 0.5, -pi/2;
            0.4+0.4*5, -0.4, 0.35, 0;
            0.4+0.4*6, -0.4, 0.2, pi/2]';

safe_regions = iris.TerrainRegion.empty();
[A, b] = poly2lincon([-1, 0.05, 0.05, -1], [-1, -1, 1, 1]);
[A, b] = convert_to_cspace(A, b);
safe_regions(1) = iris.TerrainRegion(A, b, [], [], [0;0;0], [0;0;1]);
[A, b] = poly2lincon(max(box_tops(1,:))+[0.35, 1, 1, 0.35], [-1, -1, 1, 1]);
[A, b] = convert_to_cspace(A, b);
safe_regions(2) = iris.TerrainRegion(A, b, [], [], [0;0;0], [0;0;1]);

for j = 1:size(box_tops, 2)
  rpy = [0;-15*pi/180;box_tops(4,j)];
  b = RigidBodyBox(box_size, box_tops(1:3,j) + [0;0;-box_size(3)/2], rpy);
  r = r.addGeometryToBody('world', b);
  offset = rpy2rotmat(rpy) * [0;0;box_size(3)/2];
  [A, b] = poly2lincon(box_tops(1,j) + offset(1) + [-0.03, 0.03, 0.03, -0.03],...
                       box_tops(2,j) + offset(2) + [-0.11, -0.11, 0.11, 0.11]);
  [A, b] = convert_to_cspace(A, b);
  normal = rpy2rotmat(rpy) * [0;0;1];
  safe_regions(end+1) = iris.TerrainRegion(A, b, [], [], box_tops(1:3,j), normal);
end
r = r.compile();
height_map = RigidBodyHeightMapTerrain.constructHeightMapFromRaycast(r,x0(1:nq),-1:.015:4,-1:.015:1,10);
r = r.setTerrain(height_map).compile();
r.default_walking_params.drake_min_hold_time = 1.0;

v = r.constructVisualizer();
v.display_dt = 0.01;

footstep_plan = r.planFootsteps(x0(1:nq), struct('right',[max(box_tops(1,:))+0.4;-0.13;0;0;0;0],...
                                                 'left', [max(box_tops(1,:))+0.4;0.13;0;0;0;0]),...
                                safe_regions,...
                                struct('step_params', struct('max_forward_step', 0.4,...
                                                             'nom_forward_step', 0.05,...
                                                             'nom_upward_step', 0.25,...
                                                             'nom_downward_step', 0.15,...
                                                             'max_num_steps', 13)));
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

[ytraj, com, rms_com] = atlasUtil.simulateWalking(r, walking_plan);

v.playback(ytraj, struct('slider', true));

if ~rangecheck(rms_com, 0, 0.005);
  error('Drake:runAtlasWalkingTiltedBlocks:BadCoMTracking', 'Center-of-mass during execution differs substantially from the plan.');
end

end

function [A, b] = convert_to_cspace(A, b)
  A = [A, zeros(size(A, 1), 1);
       zeros(2, size(A, 2) + 1)];
  A(end-1,3) = 1;
  A(end,3) = -1;
  b = [b; 0; 0];
end

% TIMEOUT 1000