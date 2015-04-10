function body_motions = getLinkConstraints(obj, foot_origin_knots, zmptraj, supports, support_times, options)

if nargin < 6
  options = struct();
end
options = applyDefaults(options, struct('pelvis_height_above_sole', obj.default_walking_params.pelvis_height_above_foot_sole, 'debug', true));

body_motions = BodyMotionData.empty();

if options.debug
  figure(321)
  clf
  subplot 211
  hold on
end

foot_pp = struct('right', {[]}, 'left', {[]});

if options.debug
  lcmgl = LCMGLClient('swing_traj');
end

for f = {'right', 'left'}
  foot = f{1};
  if options.debug
    if strcmp(foot, 'right')
      lcmgl.glColor3f(0.2,0.8,0.2);
    else
      lcmgl.glColor3f(0.8,0.2,0.2);
    end
  end
  foot_states = [foot_origin_knots.(foot)];
  frame_id = obj.foot_frame_id.(foot);
  body_ind = obj.getFrame(frame_id).body_ind;
  ts = [foot_origin_knots.t];

  if size(foot_states, 1) == 6
    body_motions(end+1) = BodyMotionData.from_body_xyzexp(body_ind, ts, foot_states);
  elseif size(foot_states, 1) == 12
    body_motions(end+1) = BodyMotionData.from_body_xyzexp_and_xyzexpdot(body_ind, ts, foot_states(1:6,:), foot_states(7:12,:));
  end
  body_motions(end).in_floating_base_nullspace = true(1, numel(ts));
  foot_pp.(foot) = body_motions(end).getPP();

  if options.debug
    foot_poses_exp = foot_states(1:6,:);
    tsample = linspace(ts(1), ts(end), 200);
    xs = ppval(foot_pp.(foot), tsample);
    foot_deriv_pp = fnder(foot_pp.(foot));
    xds = ppval(foot_deriv_pp, tsample);
    figure(321)
    plot(tsample, xs(3,:), 'b.-')
    xlim([0, ts(end)])

    lcmgl.glPointSize(5);
    lcmgl.points(xs(1,:), xs(2,:), xs(3,:));

    for j = 1:length(ts)
      lcmgl.sphere(foot_poses_exp(1:3,j), 0.02, 10, 10);
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

  toe_off_allowed = [foot_origin_knots.toe_off_allowed];
  body_motions(end).toe_off_allowed = [toe_off_allowed.(foot)];
end

if options.debug
  zmps = zmptraj.eval(tsample);
  plot(tsample, zmps(2,:), 'm.-');
  lcmgl.switchBuffers();
end

pelvis_reference_height = zeros(1,length(support_times));

lfoot_body_motion = body_motions([body_motions.body_id] == obj.foot_body_id.left);
rfoot_body_motion = body_motions([body_motions.body_id] == obj.foot_body_id.right);
lfoot_des = lfoot_body_motion.eval(0);
rfoot_des = rfoot_body_motion.eval(0);
pelvis_reference_height(1) = min(lfoot_des(3),rfoot_des(3));

T_sole_frame = struct('right', obj.getFrame(obj.foot_frame_id.right).T,...
           'left', obj.getFrame(obj.foot_frame_id.left).T);

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

  lfoot_orig = lfoot_body_motion.eval(t);
  T_l_orig = poseQuat2tform([lfoot_orig(1:3); expmap2quat(lfoot_orig(4:6))]);
  lsole_des = T_l_orig * T_sole_frame.left;

  rfoot_orig = rfoot_body_motion.eval(t);
  T_r_orig = poseQuat2tform([rfoot_orig(1:3); expmap2quat(rfoot_orig(4:6))]);
  rsole_des = T_r_orig * T_sole_frame.right;

  lfoot_orig_next = lfoot_body_motion.eval(t_next);
  T_l_orig_next = poseQuat2tform([lfoot_orig_next(1:3); expmap2quat(lfoot_orig_next(4:6))]);
  lsole_des_next = T_l_orig_next * T_sole_frame.left;

  rfoot_orig_next = rfoot_body_motion.eval(t_next);
  T_r_orig_next = poseQuat2tform([rfoot_orig_next(1:3); expmap2quat(rfoot_orig_next(4:6))]);
  rsole_des_next = T_r_orig_next * T_sole_frame.right;

  step_height_delta_threshold = 0.025; % cm, min change in height to classify step up/down
  step_up_pelvis_shift = 0.03; % cm
  if isDoubleSupport && nextIsDoubleSupport
    pelvis_reference_height(i+1) = pelvis_reference_height(i);
  elseif isDoubleSupport && nextIsLeftSupport
    if lsole_des_next(3) > rsole_des(3) + step_height_delta_threshold
      % stepping up with left foot
      pelvis_reference_height(i+1) = lsole_des_next(3)-step_up_pelvis_shift;
    else
      pelvis_reference_height(i+1) = lsole_des_next(3);
    end
  elseif isDoubleSupport && nextIsRightSupport

    if rsole_des_next(3) > lsole_des(3) + step_height_delta_threshold
      % stepping up with right foot
      pelvis_reference_height(i+1) = rsole_des_next(3)-step_up_pelvis_shift;
    else
      pelvis_reference_height(i+1) = rsole_des_next(3);
    end
  elseif isLeftSupport && nextIsDoubleSupport 
    if rsole_des_next(3) < lsole_des(3) - step_height_delta_threshold
      % stepping down with right foot
      pelvis_reference_height(i+1) = rsole_des_next(3)-step_up_pelvis_shift;
    else
      pelvis_reference_height(i+1) = lsole_des(3);
    end
  elseif isRightSupport && nextIsDoubleSupport 
    if lsole_des_next(3) < rsole_des(3) - step_height_delta_threshold
      % stepping down with left foot
      pelvis_reference_height(i+1) = lsole_des_next(3)-step_up_pelvis_shift;
    else
      pelvis_reference_height(i+1) = rsole_des(3);
    end
  end
end

% Now set the pelvis reference
pelvis_body = obj.findLinkId('pelvis');
pelvis_ts = support_times;
rpos = ppval(foot_pp.right, pelvis_ts);
lpos = ppval(foot_pp.left, pelvis_ts);

rexp = rpos(4:6,:);
lexp = lpos(4:6,:);
for j = 1:size(rexp, 2)
  rq = expmap2quat(rexp(:,j));
  lq = expmap2quat(lexp(:,j));
  lq = lq * sign(lq' * rq);
  lexp(:,j) = quat2expmap(lq);
end
pelvis_exp = [zeros(2, size(rexp, 2));
              mean([rexp(3,:); lexp(3,:)], 1)];

pelvis_poses_exp = [nan(2, size(rpos, 2));
                pelvis_reference_height + options.pelvis_height_above_sole;
                pelvis_exp];

pp = foh(pelvis_ts, pelvis_poses_exp);
pelvis_dposes = ppval(fnder(pp, 1), pelvis_ts);
body_motions(end+1) = BodyMotionData.from_body_xyzexp_and_xyzexpdot(pelvis_body, pelvis_ts, pelvis_poses_exp, pelvis_dposes);

if options.debug
  pp = body_motions(end).getPP();
  tt = linspace(pelvis_ts(1), pelvis_ts(end), 100);
  ps = ppval(pp, tt);
  figure(25)
  plot(tt, ps);
end
