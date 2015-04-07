function [zmp_knots, foot_origin_knots] = planZMPTraj(biped, q0, footsteps, options)
% function [zmptraj, link_constraints, support_times, supports] = planZMPTraj(biped, q0, footsteps, options)
% Plan the trajectories of the ZMP and the feet in order to follow the given footsteps
% @param q0 the initial configuration vector
% @param footsteps a list of Footstep objects
% @option t0 the initial time offset of the trajectories to be generated (default: 0)
% @option first_step_hold_s the number of seconds to wait before lifting the first foot (default: 1)

if nargin < 4; options = struct(); end

if ~isfield(options, 't0'); options.t0 = 0; end
if ~isfield(options, 'first_step_hold_s'); options.first_step_hold_s = 1.5; end

typecheck(biped,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});
typecheck(q0,'numeric');
sizecheck(q0,[biped.getNumPositions,1]);

is_right_foot = footsteps(1).frame_id == biped.foot_frame_id.right;

foot0 = feetPosition(biped, q0);
foot0.right(6) = foot0.left(6) + angleDiff(foot0.left(6), foot0.right(6));

steps.right = footsteps([footsteps.frame_id] == biped.foot_frame_id.right);
steps.left = footsteps([footsteps.frame_id] == biped.foot_frame_id.left);
steps.right(1).pos = foot0.right;
steps.left(1).pos = foot0.left;

for f = {'left', 'right'}
  foot = f{1};
  for k = 1:(length(steps.(foot))-1)
    p1 = steps.(foot)(k).pos;
    p2 = steps.(foot)(k+1).pos;
    steps.(foot)(k+1).pos = [p2(1:3); p1(4:6) + angleDiff(p1(4:6), p2(4:6))];
  end
end

rfoot_body_idx = biped.getFrame(biped.foot_frame_id.right).body_ind;
lfoot_body_idx = biped.getFrame(biped.foot_frame_id.left).body_ind;
zmp0 = [];
initial_supports = [];
if steps.right(1).is_in_contact
  z = steps.right(1).pos;
  zmp0(:,end+1) = z(1:2);
  initial_supports(end+1) = rfoot_body_idx;
end
if steps.left(1).is_in_contact
  z = steps.left(1).pos;
  zmp0(:,end+1) = z(1:2);
  initial_supports(end+1) = lfoot_body_idx;
end
zmp0 = mean(zmp0, 2);
supp0 = RigidBodySupportState(biped, initial_supports);
zmp_knots = struct('t', options.t0, 'zmp', zmp0, 'supp', supp0);

foot_origin_knots = struct('t', options.t0, ...
  'right', zeros(12,1),...
  'left', zeros(12,1),...
  'is_liftoff', false,...
  'is_landing', false,...
  'toe_off_allowed', struct('right', false, 'left', false));
for f = {'right', 'left'}
  foot = f{1};
  frame_id = biped.foot_frame_id.(foot);
  T = biped.getFrame(frame_id).T;
  sole_pose = steps.(foot)(1).pos;
  Tsole = [rpy2rotmat(sole_pose(4:6)), sole_pose(1:3); 0 0 0 1];
  Torig = Tsole / T;
  foot_origin_knots.(foot) = [Torig(1:3,4); rotmat2rpy(Torig(1:3,1:3)); zeros(6,1)];
end

istep = struct('right', 1, 'left', 1);
is_first_step = true;

while 1
  if is_right_foot
    sw_foot = 'right'; % moving (swing) foot
    st_foot = 'left'; % stance foot
  else
    sw_foot = 'left';
    st_foot = 'right';
  end
  sw0 = steps.(sw_foot)(istep.(sw_foot));
  sw1 = steps.(sw_foot)(istep.(sw_foot)+1);
  st = steps.(st_foot)(istep.(st_foot));

  if is_first_step
    initial_hold = options.first_step_hold_s;
%     sw1.walking_params.drake_min_hold_time = options.first_step_hold_s;
    is_first_step = false;
    sw1.walking_params.step_speed = sw1.walking_params.step_speed / 2;
  else
    initial_hold = 0;
  end
  if istep.left == length(steps.left) || istep.right == length(steps.right)
    % this is the last swing, so slow down
    sw1.walking_params.step_speed = sw1.walking_params.step_speed / 2;
  end

  [new_foot_knots, new_zmp_knots] = planSwingPitched(biped, st, sw0, sw1, initial_hold);
  t0 = foot_origin_knots(end).t;
  for k = 1:length(new_foot_knots)
    new_foot_knots(k).t = new_foot_knots(k).t + t0;
  end
  for k = 1:length(new_zmp_knots)
    new_zmp_knots(k).t = new_zmp_knots(k).t + t0;
  end

  % % Allow toe-off two knot points earlier than the beginning of the swing. This is
  % % necessary to allow toe-off to start earlier than halfway through the zmp shift
  % if length(foot_origin_knots) > 2
  %   foot_origin_knots(end-2).toe_off_allowed = new_foot_knots(1).toe_off_allowed;
  % end
  % if length(foot_origin_knots) > 1
  %   foot_origin_knots(end-1).toe_off_allowed = new_foot_knots(1).toe_off_allowed;
  % end
  foot_origin_knots(end).toe_off_allowed.(sw_foot) = true;
  foot_origin_knots = [foot_origin_knots, new_foot_knots];
  zmp_knots = [zmp_knots, new_zmp_knots];

  istep.(sw_foot) = istep.(sw_foot) + 1;

  is_right_foot = ~is_right_foot;
  if istep.left == length(steps.left) && istep.right == length(steps.right)
    break
  end
end

% add a segment at the end to recover
t0 = foot_origin_knots(end).t;
foot_origin_knots(end+1) = foot_origin_knots(end);
foot_origin_knots(end).t = t0 + 1.5;
zmpf = mean([steps.right(end).pos(1:2), steps.left(end).pos(1:2)], 2);
zmp_knots(end+1) =  struct('t', foot_origin_knots(end-1).t, 'zmp', zmpf, 'supp', RigidBodySupportState(biped, [rfoot_body_idx, lfoot_body_idx]));
zmp_knots(end+1) =  struct('t', foot_origin_knots(end).t, 'zmp', zmpf, 'supp', RigidBodySupportState(biped, [rfoot_body_idx, lfoot_body_idx]));

% dynamic_footstep_plan = DynamicFootstepPlan(biped, zmp_knots, foot_origin_knots);
end
