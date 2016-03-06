classdef BodyMotionData
  % Data structure describing the desired motion of a single body on the
  % robot. The QPLocomotionPlan will call this class's slice() method to get a
  % snapshot of the desired body motion, which will be sent to the controller
  % as part of the QP controller input.
  % 
  % The desired body motion is expressed as a piecewise cubic polynomial. 
  properties
    ts;  % Times of the breakpoints in the cubic polynomial
    coefs; % Coefficients of the cubic polynomial. The format is [x; y; z; w1; w2; w3] where w(1:3) is the exponential map representation of the orientation. 
    body_id; % Body or frame ID to track
    toe_off_allowed; % A flag used by the QPLocomotionPlan to determine whether to engage toe-only support when walking. Set this to False unless you have some reason not to.
    in_floating_base_nullspace; % Whether to explicitly disallow floating base acceleration as a method for accelerating this body in the controller. We set this for the legs so that, for example, if the leg falls behind its plan, the controller doesn't accelerate the pelvis forward to compensate, which can cause a fall. 
    control_pose_when_in_contact; % Whether to attempt to control the pose of this body even when it's in contact with the world. This is typically false, but we've set it to true when using the pelvis as a support. 
    quat_task_to_world % A rotation transformation, the coefs is specified in the task frame
    translation_task_to_world % translation from task frame to the world frame
    xyz_kp_multiplier % A 3 x 1 vector. The multiplier for the Kp gain on xyz position error of the body, default to [1;1;1]
    xyz_damping_ratio_multiplier % A 3 x 1 vector. The multiplier for the damping ratio on xyz velocity error of the body, default to [1;1;1]
    expmap_kp_multiplier % A positive scalar. The multiplier for the Kp gain on exponential map error of body orientation, default to 1
    expmap_damping_ratio_multiplier % A positive scalar. The multiplier for the damping ratio on exponential map velocity error of body orientation, default to 1
    weight_multiplier % A 6 x 1 vector. The multiplier for twist_dot (angular_aceleration, cartesian_acceleration) weight in the QP controller, default to ones(6,1);
  end

  methods
    function obj = BodyMotionData(body_id, ts)
      obj.ts = ts;
      obj.body_id = body_id;
      obj.toe_off_allowed = false(1, numel(ts));
      obj.in_floating_base_nullspace = false(1, numel(ts));
      obj.control_pose_when_in_contact = false(1, numel(ts));
      obj.coefs = zeros(6, numel(ts), 4);
      obj.quat_task_to_world = [1;0;0;0];
      obj.translation_task_to_world = zeros(3,1);
      obj.xyz_kp_multiplier = [1;1;1];
      obj.xyz_damping_ratio_multiplier = [1;1;1];
      obj.expmap_kp_multiplier = 1;
      obj.expmap_damping_ratio_multiplier = 1;
      obj.weight_multiplier = ones(6,1);
    end

    function t_ind = findTInd(obj, t)
      if t < obj.ts(1)
        t_ind = 1;
      elseif t >= obj.ts(end)
        t_ind = length(obj.ts) - 1;
      else
        t_ind = find(obj.ts <= t, 1, 'last');
      end
    end

    function [x, xd, xdd] = eval(obj, t)
      t_ind = obj.findTInd(t);
      [x, xd, xdd] = obj.evalAtInd(t_ind, t);
    end

    function body_motion_slice = sliceAtTime(obj, t)
      body_motion_slice = obj.slice(obj.findTInd(t));
    end

    function body_motion_slice = slice(obj, t_ind)
      if (length(obj.ts) < 3)
        slice_t_inds = min([t_ind, t_ind+1], [length(obj.ts), length(obj.ts)]);
        slice_t_inds = max(slice_t_inds, [1, 1]);
        slice_coef_t_inds = t_ind;
      else
        slice_t_inds = min([t_ind, t_ind+1, t_ind+2], [length(obj.ts), length(obj.ts), length(obj.ts)]);
        slice_t_inds = max(slice_t_inds, [1, 1, 1]);
        slice_coef_t_inds = min(slice_t_inds(1:2), [size(obj.coefs, 2), size(obj.coefs, 2)]); 
      end
      % lookahead a slice
      
      body_motion_slice = struct('body_id', obj.body_id,...
                                 'ts', obj.ts(slice_t_inds),...
                                 'coefs', obj.coefs(:,slice_coef_t_inds,:),...
                                 'toe_off_allowed', obj.toe_off_allowed(t_ind),...
                                 'in_floating_base_nullspace', obj.in_floating_base_nullspace(t_ind),...
                                 'control_pose_when_in_contact', obj.control_pose_when_in_contact(t_ind),...
                                 'quat_task_to_world',obj.quat_task_to_world,...
                                 'translation_task_to_world',obj.translation_task_to_world,...
                                 'xyz_kp_multiplier',obj.xyz_kp_multiplier,...
                                 'xyz_damping_ratio_multiplier',obj.xyz_damping_ratio_multiplier,...
                                 'expmap_kp_multiplier',obj.expmap_kp_multiplier,...
                                 'expmap_damping_ratio_multiplier',obj.expmap_damping_ratio_multiplier,...
                                 'weight_multiplier',obj.weight_multiplier);
    end

    function pp = getPP(obj)
      pp = mkpp(obj.ts, obj.coefs, size(obj.coefs, 1));
    end
  end

  methods(Static)
    function obj = from_body_poses(body_id, ts, xyz_rpy)
      % Maintained for backwards compatibility, assumed to be xyzrpy
      obj = BodyMotionData.from_body_xyzrpy(body_id, ts, xyz_rpy);
    end

    function obj = from_body_poses_and_velocities(body_id, ts, xyz_rpy, xyz_rpydot)
      % Maintained for backwards compatibility, assumed to be xyzrpy
      quat = zeros(4, size(xyz_rpy, 2));
      quatdot = zeros(4, size(xyz_rpy, 2));
      for j = 1:size(xyz_rpy, 2)
        quat(:,j) = rpy2quat(xyz_rpy(4:6,j));
        angvel = rpydot2angularvel(xyz_rpy(4:6,j), xyz_rpydot(4:6,j));
        M = angularvel2quatdotMatrix(quat(:,j));
        quatdot(:,j) = M * angvel;
      end
      [expmap, expmapdot] = quat2expmapSequence(quat, quatdot);
      xyz_exp = [xyz_rpy(1:3,:); expmap];
      xyz_expdot = [xyz_rpydot(1:3,:); expmapdot];
      obj = BodyMotionData.from_body_xyzexp_and_xyzexpdot(body_id, ts, xyz_exp, xyz_expdot);
    end

    function obj = from_body_xyzrpy(body_id, ts, poses)
      quat = zeros(4, size(poses, 2));
      for j = 1:size(poses, 2)
        quat(:,j) = rpy2quat(poses(4:6,j));
      end
      expmap = quat2expmapSequence(quat, zeros(size(quat)));
      xyz_exp = [poses(1:3,:); expmap];
      obj = BodyMotionData.from_body_xyzexp(body_id, ts, xyz_exp);
    end

    function obj = from_body_xyzquat(body_id, ts, xyz_quat)
      xyz_exp = zeros(6, size(xyz_quat, 2));
      xyz_exp(1:3,:) = xyz_quat(1:3,:);
      expmap = quat2expmapSequence(xyz_quat(4:7,:), zeros(4, size(xyz_quat,2)));
      xyz_exp(4:6,:) = expmap;
      obj = BodyMotionData.from_body_xyzexp(body_id, ts, xyz_exp);
    end

    function obj = from_body_xyzquat_and_xyzquatdot(body_id, ts, xyzquat, xyzquatdot)
      [expmap, expmapdot] = quat2expmapSequence(xyzquat(4:7,:), xyzquatdot(4:7,:));
      xyz_exp = [xyzquat(1:3,:); expmap];
      xyz_expdot = [xyzquatdot(1:3,:); expmapdot];
      obj = BodyMotionData.from_body_xyzexp_and_xyzexpdot(body_id, ts, xyz_exp, xyz_expdot);
    end

    function obj = from_body_xyzexp(body_id, ts, poses_exp)
      % [xyz; exp]
      for j = 2:size(poses_exp, 2)
        w2 = closestExpmap(poses_exp(4:6,j-1), poses_exp(4:6,j));
        poses_exp(4:6,j) = w2;
      end
      obj = BodyMotionData(body_id, ts);
      pp = pchip(ts, poses_exp);
      [~, obj.coefs, l, k, d] = unmkpp(pp);
      obj.coefs = reshape(obj.coefs, [d, l, k]);
    end

    function obj = from_body_xyzexp_and_xyzexpdot(body_id, ts, poses_exp, dposes_exp)
      for j = 2:size(poses_exp, 2)
        [w2, dw2] = closestExpmap(poses_exp(4:6,j-1), poses_exp(4:6,j));
        poses_exp(4:6,j) = w2;
        dposes_exp(4:6,j) = dw2 * dposes_exp(4:6,j);
      end
      obj = BodyMotionData(body_id, ts);
      pp = pchipDeriv(ts, poses_exp, dposes_exp);
      [~, obj.coefs, l, k, d] = unmkpp(pp);
      obj.coefs = reshape(obj.coefs, [d, l, k]);
    end
  end
  
  methods (Access=private)
    function [x, xd, xdd] = evalAtInd(obj, t_ind, t)
      t_rel = t - obj.ts(t_ind);
      x = obj.coefs(:,t_ind,1)*t_rel^3 + obj.coefs(:,t_ind,2)*t_rel^2 + obj.coefs(:,t_ind,3)*t_rel + obj.coefs(:,t_ind,4);
      xd = 3*obj.coefs(:,t_ind,1)*t_rel^2 + 2*obj.coefs(:,t_ind,2)*t_rel + obj.coefs(:,t_ind,3);
      xdd = 6*obj.coefs(:,t_ind,1)*t_rel + 2*obj.coefs(:,t_ind,2);
    end
  end
end

