classdef BodyMotionData
  properties
    ts;
    coefs;
    body_id;
    toe_off_allowed;
    in_floating_base_nullspace;
    control_pose_when_in_contact;
    use_spatial_velocity
    quat_task_to_world % A rotation transformation, the coefs is specified in the task frame
    translation_task_to_world % translation from task frame to the world frame
  end

  methods
    function obj = BodyMotionData(body_id, ts)
      obj.ts = ts;
      obj.body_id = body_id;
      obj.toe_off_allowed = false(1, numel(ts));
      obj.in_floating_base_nullspace = false(1, numel(ts));
      obj.control_pose_when_in_contact = false(1, numel(ts));
      obj.coefs = zeros(6, numel(ts), 4);
      obj.use_spatial_velocity = false;
      obj.quat_task_to_world = [1;0;0;0];
      obj.translation_task_to_world = zeros(3,1);
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

    function [x, xd, xdd] = evalAtInd(obj, t_ind, t)
      t_rel = t - obj.ts(t_ind);
      x = obj.coefs(:,t_ind,1)*t_rel^3 + obj.coefs(:,t_ind,2)*t_rel^2 + obj.coefs(:,t_ind,3)*t_rel + obj.coefs(:,t_ind,4);
      xd = 3*obj.coefs(:,t_ind,1)*t_rel^2 + 2*obj.coefs(:,t_ind,2)*t_rel + obj.coefs(:,t_ind,3);
      xdd = 6*obj.coefs(:,t_ind,1)*t_rel + 2*obj.coefs(:,t_ind,2);
    end

    function body_motion_slice = sliceAtTime(obj, t)
      body_motion_slice = obj.slice(obj.findTInd(t));
    end

    function body_motion_slice = slice(obj, t_ind)
      slice_t_inds = min([t_ind, t_ind+1], [length(obj.ts), length(obj.ts)]);
      slice_t_inds = max(slice_t_inds, [1, 1]);
      body_motion_slice = struct('body_id', obj.body_id,...
                                 'ts', obj.ts(slice_t_inds),...
                                 'coefs', obj.coefs(:,t_ind,:),...
                                 'toe_off_allowed', obj.toe_off_allowed(t_ind),...
                                 'in_floating_base_nullspace', obj.in_floating_base_nullspace(t_ind),...
                                 'control_pose_when_in_contact', obj.control_pose_when_in_contact(t_ind),...
                                 'use_spatial_velocity',obj.use_spatial_velocity,...
                                 'quat_task_to_world',obj.quat_task_to_world,...
                                 'translation_task_to_world',obj.translation_task_to_world);
    end

    function pp = getPP(obj)
      pp = mkpp(obj.ts, obj.coefs, size(obj.coefs, 1));
    end

    function obj = extend(obj, new_body_motion_data)
      if obj.ts(end) ~= new_body_motion_data.ts(1)
        error('Drake:BodyMotionData:BadExtend', 'final time of first arg should match initial time of second arg');
      end
      if size(obj.coefs, 1) ~= size(new_body_motion_data.coefs, 1)
        error('Drake:BodyMotionData:BadExtend', 'dimension of coefs should match');
      end
      if obj.body_id ~= new_body_motion_data.body_id
        error('Drake:BodyMotionData:BadExtend', 'body_ids must match');
      end
      if obj.use_spatial_velocity ~= new_body_motion_data.use_spatial_velocity
        error('Drake:BodyMotionData:BadExtend', 'use_spatial_velocity flag must match');
      end
      if any(obj.quat_task_to_world ~= new_body_motion_data.quat_task_to_world)
        error('Drake:BodyMotionData:BadExtend', 'quat_task_to_world must match');
      end
      if any(obj.translation_task_to_world ~= new_body_motion_data.translation_task_to_world)
        error('Drake:BodyMotionData:BadExtend', 'translation_task_to_world must match');
      end
      nts = numel(obj.ts) - 1;
      obj.ts = [obj.ts(1:nts), new_body_motion_data.ts];
      obj.toe_off_allowed = [obj.toe_off_allowed(1:nts), new_body_motion_data.toe_off_allowed];
      obj.in_floating_base_nullspace = [obj.in_floating_base_nullspace(1:nts), new_body_motion_data.in_floating_base_nullspace];
      obj.control_pose_when_in_contact = [obj.control_pose_when_in_contact(1:nts), new_body_motion_data.control_pose_when_in_contact];
    end
  end

  methods(Static)
    function obj = from_body_poses(body_id, ts, poses)
      % [xyz; rpy]
      obj = BodyMotionData(body_id, ts);
      pp = pchip(ts, poses);
      [~, obj.coefs, l, k, d] = unmkpp(pp);
      obj.coefs = reshape(obj.coefs, [d, l, k]);
    end

    function obj = from_body_poses_and_velocities(body_id, ts, poses, dposes)
      % [xyz; rpy] and [xyzdot; rpydot]
      obj = BodyMotionData(body_id, ts);

      pp = pchipDeriv(ts, poses, dposes);
      [~, obj.coefs, l, k, d] = unmkpp(pp);
      obj.coefs = reshape(obj.coefs, [d, l, k]);
    end

    function obj = from_body_xyzrpy_pp(body_id, pp)
      [ts, coefs, l, k, d] = unmkpp(pp);
      coefs = reshape(coefs, [d, l, k]);
      obj = BodyMotionData(body_id, ts);
      if size(coefs, 3) < 4
        coefs = cat(3, zeros([6, size(coefs, 2), 4 - size(coefs, 3)]), coefs);
      elseif size(coefs, 3) > 4
        error('Drake:BodyMotionData:SplineDegreeTooHigh', 'BodyMotionData only supports splines up to degree 3 (cubic)');
      end
      obj.coefs = coefs;
    end

    function obj = from_body_xyzexp(body_id, ts, poses_exp)
      % [xyz; exp]
      obj = BodyMotionData(body_id, ts);
      obj.use_spatial_velocity = true;
      pp = pchip(ts, poses_exp);
      [~, obj.coefs, l, k, d] = unmkpp(pp);
      obj.coefs = reshape(obj.coefs, [d, l, k]);
    end

    function obj = from_body_xyzexp_and_xyzexpdot(body_id, ts, poses_exp, dposes_exp)
      obj = BodyMotionData(body_id, ts);
      obj.use_spatial_velocity = true;
      pp = pchipDeriv(ts, poses_exp, dposes_exp);
      [~, obj.coefs, l, k, d] = unmkpp(pp);
      obj.coefs = reshape(obj.coefs, [d, l, k]);
    end
  end
end

