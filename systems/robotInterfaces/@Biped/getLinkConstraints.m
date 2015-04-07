function link_constraints = getLinkConstraints(obj, foot_origin_knots, zmptraj, supports, support_times, options)

if nargin < 6
  options = struct();
end
options = applyDefaults(options, struct('pelvis_height_above_sole', obj.default_walking_params.pelvis_height_above_foot_sole, 'debug', true));

link_constraints = struct('link_ndx',{}, 'pt', {}, 'ts', {}, 'poses', {}, 'dposes', {}, 'contact_break_indices', {}, 'coefs', {}, 'toe_off_allowed', {}, 'in_floating_base_nullspace', {});

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
  foot_poses = foot_states(1:6,:);
  frame_id = obj.foot_frame_id.(foot);
  body_ind = obj.getFrame(frame_id).body_ind;
  foot_poses(4:6,:) = unwrap(foot_poses(4:6,:), [], 2);
  ts = [foot_origin_knots.t];

  if size(foot_states, 1) == 6
    foot_pp.(foot) = pchip(ts, foot_poses);
    foot_dposes = ppval(fnder(foot_pp.(foot), 1), ts);
  elseif size(foot_states, 1) == 12
    foot_dposes = foot_states(7:12,:);
    foot_pp.(foot) = pchipDeriv(ts, foot_poses, foot_dposes);
  end

  if options.debug
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
      lcmgl.sphere(foot_poses(1:3,j), 0.02, 10, 10);
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

  % Compute cubic polynomial coefficients to save work in the controller
  [~, coefs, l, k, d] = unmkpp(foot_pp.(foot));
  coefs = reshape(coefs, [d, l, k]);
  assert(k == 4, 'expected a piecewise cubic polynomial');
  toe_off_allowed = [foot_origin_knots.toe_off_allowed];
  link_constraints(end+1) = struct('link_ndx', body_ind, 'pt', [0;0;0], 'ts', ts, 'poses', foot_poses, 'dposes', foot_dposes, 'contact_break_indices', find([foot_origin_knots.is_liftoff]), 'coefs', coefs, 'toe_off_allowed', [toe_off_allowed.(foot)], 'in_floating_base_nullspace', true);
end

if options.debug
  zmps = zmptraj.eval(tsample);
  plot(tsample, zmps(2,:), 'm.-');
  lcmgl.switchBuffers();
end

pelvis_reference_height = zeros(1,length(support_times));

lfoot_link_con_ind = [link_constraints.link_ndx]==obj.foot_body_id.left;
rfoot_link_con_ind = [link_constraints.link_ndx]==obj.foot_body_id.right;
lfoot_des = evaluateSplineInLinkConstraints(0,link_constraints,lfoot_link_con_ind);
rfoot_des = evaluateSplineInLinkConstraints(0,link_constraints,rfoot_link_con_ind);
pelvis_reference_height(1) = min(lfoot_des(3),rfoot_des(3));

T = obj.getFrame(obj.foot_frame_id.right).T;
% Torig * T = Tsole
rfoot_sole_des = tform2poseRPY(poseRPY2tform(rfoot_des) * T);
pelvis_height_above_foot_origin = options.pelvis_height_above_sole + (rfoot_sole_des(3) - rfoot_des(3));

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
  lfoot_des = evaluateSplineInLinkConstraints(t,link_constraints,lfoot_link_con_ind);
  rfoot_des = evaluateSplineInLinkConstraints(t,link_constraints,rfoot_link_con_ind);
  lfoot_des_next = evaluateSplineInLinkConstraints(t_next,link_constraints,lfoot_link_con_ind);
  rfoot_des_next = evaluateSplineInLinkConstraints(t_next,link_constraints,rfoot_link_con_ind);

  step_height_delta_threshold = 0.025; % cm, min change in height to classify step up/down
  step_up_pelvis_shift = 0.03; % cm
  if isDoubleSupport && nextIsDoubleSupport
    pelvis_reference_height(i+1) = pelvis_reference_height(i);
  elseif isDoubleSupport && nextIsLeftSupport
    if lfoot_des_next(3) > rfoot_des(3) + step_height_delta_threshold
      % stepping up with left foot
      pelvis_reference_height(i+1) = lfoot_des_next(3)-step_up_pelvis_shift;
    else
      pelvis_reference_height(i+1) = lfoot_des_next(3);
    end
  elseif isDoubleSupport && nextIsRightSupport

    if rfoot_des_next(3) > lfoot_des(3) + step_height_delta_threshold
      % stepping up with right foot
      pelvis_reference_height(i+1) = rfoot_des_next(3)-step_up_pelvis_shift;
    else
      pelvis_reference_height(i+1) = rfoot_des_next(3);
    end
  elseif isLeftSupport && nextIsDoubleSupport 
    if rfoot_des_next(3) < lfoot_des(3) - step_height_delta_threshold
      % stepping down with right foot
      pelvis_reference_height(i+1) = rfoot_des_next(3)-step_up_pelvis_shift;
    else
      pelvis_reference_height(i+1) = lfoot_des(3);
    end
  elseif isRightSupport && nextIsDoubleSupport 
    if lfoot_des_next(3) < rfoot_des(3) - step_height_delta_threshold
      % stepping down with left foot
      pelvis_reference_height(i+1) = lfoot_des_next(3)-step_up_pelvis_shift;
    else
      pelvis_reference_height(i+1) = rfoot_des(3);
    end
  end
end

% Now set the pelvis reference
pelvis_body = obj.findLinkId('pelvis');
pelvis_ts = support_times;
rpos = ppval(foot_pp.right, pelvis_ts);
lpos = ppval(foot_pp.left, pelvis_ts);
pelvis_poses = [nan(2, size(rpos, 2));
                pelvis_reference_height + pelvis_height_above_foot_origin;
                zeros(2,size(rpos, 2));
                unwrap(angleAverage(rpos(6,:)', lpos(6,:)')')];
pp = foh(pelvis_ts, pelvis_poses);
[~,coefs, l, k, d] = unmkpp(pp);
coefs = reshape(coefs, d, l, k);
coefs = cat(3, zeros(6, size(coefs, 2), 2), coefs);
link_constraints(end+1).link_ndx = pelvis_body;
link_constraints(end).pt = [0;0;0];
link_constraints(end).ts = pelvis_ts;
link_constraints(end).poses = pelvis_poses;
link_constraints(end).coefs = coefs;
link_constraints(end).in_floating_base_nullspace = false;


% Only needed for backwards compatibility
link_constraints(1).pelvis_reference_height = pelvis_reference_height;
