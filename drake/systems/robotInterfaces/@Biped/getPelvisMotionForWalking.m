function pelvis_motion_data = getPelvisMotionForWalking(obj, x0, foot_motion_data, supports, support_times, options)
% Get the BodyMotionData for the robot's pelvis, given the BodyMotionData for its feet, plus its planned support sequence. 
typecheck(foot_motion_data, 'BodyMotionData');
typecheck(supports, 'RigidBodySupportState');
assert(isnumeric(support_times));

if nargin < 6
  options = struct();
end
options = applyDefaults(options, struct('pelvis_height_above_sole', obj.default_walking_params.pelvis_height_above_foot_sole, 'debug', false,...
  'pelvis_height_transition_knot', 1));

if options.debug
  figure(321)
  clf
  subplot 211
  hold on
end

if options.debug
  lcmgl = LCMGLClient('swing_traj');
end

pelvis_reference_height = zeros(1,length(support_times));

T_orig_to_sole = struct('right', obj.getFrame(obj.foot_frame_id.right).T,...
           'left', obj.getFrame(obj.foot_frame_id.left).T);
for j = 1:2
  frame_or_body_id = foot_motion_data(j).body_id;
  if frame_or_body_id > 0
    if frame_or_body_id == obj.foot_body_id.right
      T_frame_to_sole.right = T_orig_to_sole.right;
    else
      T_frame_to_sole.left = T_orig_to_sole.left;
    end
  else
    frame = obj.getFrame(frame_or_body_id);
    T_orig_to_frame = frame.T;
    if frame.body_ind == obj.foot_body_id.right
      T_frame_to_sole.right = T_orig_to_frame \ T_orig_to_sole.right;
    else
      T_frame_to_sole.left = T_orig_to_frame \ T_orig_to_sole.left;
    end
  end
end

if options.debug
  for i = 1:2
    ts = foot_motion_data(i).ts;
    tsample = linspace(ts(1), ts(end), 200);
    pp = foot_motion_data(i).getPP();
    xs = ppval(pp, tsample);
    foot_deriv_pp = fnder(pp);
    xds = ppval(foot_deriv_pp, tsample);
    figure(320)
    plot(tsample, xs);
    
    figure(321)
    plot(tsample, xs(3,:), 'b.-')
    xlim([0, ts(end)])

    lcmgl.glColor3f(0.2,0.9,0.2);
    lcmgl.glPointSize(5);
    lcmgl.points(xs(1,:), xs(2,:), xs(3,:));

    for j = 1:length(ts)
      pose = foot_motion_data(i).eval(ts(j));
      lcmgl.sphere(pose(1:3), 0.02, 10, 10);
    end

    vscale = 0.1;
    for j = 1:size(xs,2)
      lcmgl.line3(xs(1,j), xs(2,j), xs(3,j), ...
                  xs(1,j) + vscale*xds(1,j),...
                  xs(2,j) + vscale*xds(2,j),...
                  xs(3,j) + vscale*xds(3,j));
    end

    oldfig = gcf();
    figure(322)
    clf
    hold on
    plot(tsample, xds(1,:), 'r.-')
    plot(tsample, xds(2,:), 'g.-')
    plot(tsample, xds(3,:), 'b.-')
    sfigure(oldfig);
  end

  lcmgl.switchBuffers();
end

for j = 1:length(foot_motion_data)
  body_id = foot_motion_data(j).body_id;
  if body_id < 0
    body_id = obj.getFrame(body_id).body_ind;
  end
  if body_id == obj.foot_body_id.left
    lfoot_body_motion = foot_motion_data(j);
  elseif body_id == obj.foot_body_id.right
    rfoot_body_motion = foot_motion_data(j);
  end
end

lfoot_frame = lfoot_body_motion.eval(0);
T_l_frame = poseQuat2tform([lfoot_frame(1:3); expmap2quat(lfoot_frame(4:6))]);
lsole_des = tform2poseQuat(T_l_frame * T_frame_to_sole.left);

rfoot_frame = rfoot_body_motion.eval(0);
T_r_frame = poseQuat2tform([rfoot_frame(1:3); expmap2quat(rfoot_frame(4:6))]);
rsole_des = tform2poseQuat(T_r_frame * T_frame_to_sole.right);

pelvis_reference_height(1) = min(lsole_des(3),rsole_des(3));

for i=1:length(support_times)-1
  isDoubleSupport = any(supports(i).bodies==obj.foot_body_id.left) && any(supports(i).bodies==obj.foot_body_id.right);
  isRightSupport = ~any(supports(i).bodies==obj.foot_body_id.left) && any(supports(i).bodies==obj.foot_body_id.right);
  isLeftSupport = any(supports(i).bodies==obj.foot_body_id.left) && ~any(supports(i).bodies==obj.foot_body_id.right);
  if options.debug
    plot(support_times(i:i+1), 0.15 + 0.05*(isRightSupport|isDoubleSupport)*[1, 1], 'go:')
    plot(support_times(i:i+1), 0.15 + 0.05*(isLeftSupport|isDoubleSupport)*[1,1], 'ro:')
  end

  nextIsDoubleSupport = any(supports(i+1).bodies==obj.foot_body_id.left) && any(supports(i+1).bodies==obj.foot_body_id.right);
  nextIsRightSupport = ~any(supports(i+1).bodies==obj.foot_body_id.left) && any(supports(i+1).bodies==obj.foot_body_id.right);
  nextIsLeftSupport = any(supports(i+1).bodies==obj.foot_body_id.left) && ~any(supports(i+1).bodies==obj.foot_body_id.right);

  t = support_times(i);
  t_next = support_times(i+1);

  lfoot_frame = lfoot_body_motion.eval(t);
  T_l_frame = poseQuat2tform([lfoot_frame(1:3); expmap2quat(lfoot_frame(4:6))]);
  lsole_des = tform2poseQuat(T_l_frame * T_frame_to_sole.left);

  rfoot_frame = rfoot_body_motion.eval(t);
  T_r_frame = poseQuat2tform([rfoot_frame(1:3); expmap2quat(rfoot_frame(4:6))]);
  rsole_des = tform2poseQuat(T_r_frame * T_frame_to_sole.right);

  lfoot_frame_next = lfoot_body_motion.eval(t_next);
  T_l_frame_next = poseQuat2tform([lfoot_frame_next(1:3); expmap2quat(lfoot_frame_next(4:6))]);
  lsole_des_next = tform2poseQuat(T_l_frame_next * T_frame_to_sole.left);

  rfoot_frame_next = rfoot_body_motion.eval(t_next);
  T_r_frame_next = poseQuat2tform([rfoot_frame_next(1:3); expmap2quat(rfoot_frame_next(4:6))]);
  rsole_des_next = tform2poseQuat(T_r_frame_next * T_frame_to_sole.right);

  step_height_delta_threshold = 0.05; % cm, min change in height to classify step up/down
  step_up_pelvis_shift = 0.07; % cm
  if isDoubleSupport && nextIsDoubleSupport
    pelvis_reference_height(i+1) = pelvis_reference_height(i);
  elseif isDoubleSupport && nextIsLeftSupport
    if lsole_des_next(3) > rsole_des(3) + step_height_delta_threshold
      % has stepped up with left foot
      pelvis_reference_height(i+1) = lsole_des_next(3)-step_up_pelvis_shift;
    else
      pelvis_reference_height(i+1) = lsole_des_next(3);
    end
  elseif isDoubleSupport && nextIsRightSupport
    if rsole_des_next(3) > lsole_des(3) + step_height_delta_threshold
      % has stepped up with right foot
      pelvis_reference_height(i+1) = rsole_des_next(3)-step_up_pelvis_shift;
    else
      pelvis_reference_height(i+1) = rsole_des_next(3);
    end
  elseif isLeftSupport && nextIsDoubleSupport 
    if rsole_des_next(3) < lsole_des(3)
      % stepping down with right foot
      pelvis_reference_height(i+1) = rsole_des_next(3);
    else
      pelvis_reference_height(i+1) = lsole_des(3);
    end
  elseif isRightSupport && nextIsDoubleSupport 
    if lsole_des_next(3) < rsole_des(3) 
      % stepping down with left foot
      pelvis_reference_height(i+1) = lsole_des_next(3);
    else
      pelvis_reference_height(i+1) = rsole_des(3);
    end
  end
end



% Now set the pelvis reference
pelvis_body = obj.findLinkId('pelvis');
pelvis_ts = support_times;

% smooth commanded pelvis height and rpy with current position
q0 = x0(1:obj.getNumPositions());
kinsol = obj.doKinematics(q0);
pelvis_pos0 = obj.forwardKin(kinsol,pelvis_body,[0;0;0],1);
pelvis_height_above_sole0 = pelvis_pos0(3) - pelvis_reference_height(1);

rpos = ppval(rfoot_body_motion.getPP(), pelvis_ts);
lpos = ppval(lfoot_body_motion.getPP(), pelvis_ts);

pelvis_yaw = zeros(1, numel(pelvis_ts));

for j = 1:numel(pelvis_ts)
  rrpy = quat2rpy(expmap2quat(rpos(4:6,j)));
  lrpy = quat2rpy(expmap2quat(lpos(4:6,j)));
  pelvis_yaw(j) = angleAverage(rrpy(3), lrpy(3));
end
pelvis_yaw = unwrap(pelvis_yaw);

% Smooth commanded pelvis height between current and default
pelvis_height = pelvis_reference_height + options.pelvis_height_above_sole;
knot_idx = options.pelvis_height_transition_knot;
pelvis_height(1:knot_idx) = pelvis_reference_height(1:knot_idx) + pelvis_height_above_sole0;

pelvis_poses_rpy = [zeros(2, size(rpos, 2));
                pelvis_height;
                zeros(2, numel(pelvis_ts));
                pelvis_yaw];

% smooth desired pelvis rpy with current rpy for the first pelvis_ts time
% need to be careful to do an unwrap
pelvis_poses_rpy(4:6,1) = pelvis_pos0(4:6);
for j =4:6
  pelvis_poses_rpy(j,:) = unwrap(pelvis_poses_rpy(j,:));
end

pelvis_xyz_expmap = zeros(6,size(pelvis_poses_rpy,2));
pelvis_xyz_expmap(1:3,:) = pelvis_poses_rpy(1:3,:);
pelvis_quat = zeros(4,size(pelvis_poses_rpy,2));
for i = 1:size(pelvis_poses_rpy,2)
  pelvis_quat(:,i) = rpy2quat(pelvis_poses_rpy(4:6,i));
  pelvis_xyz_expmap(4:6,i) = quat2expmap(pelvis_quat(:,i));
  if(i>1)
    pelvis_xyz_expmap(4:6,i) = closestExpmap(pelvis_xyz_expmap(4:6,i-1),pelvis_xyz_expmap(4:6,i));
  end
end

% pp = foh(pelvis_ts, pelvis_xyz_expmap);
pelvis_motion_data = BodyMotionData.from_body_xyzexp(pelvis_body, pelvis_ts,pelvis_xyz_expmap);
pelvis_motion_data.weight_multiplier = [1;1;1;0;0;1];
if options.debug
  pp = pelvis_motion_data.getPP();
  tt = linspace(pelvis_ts(1), pelvis_ts(end), 100);
  ps = ppval(pp, tt);
  figure(25)
  plot(tt, ps);
end
