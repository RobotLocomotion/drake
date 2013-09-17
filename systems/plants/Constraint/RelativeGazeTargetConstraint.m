classdef RelativeGazeTargetConstraint < GazeTargetConstraint
  properties
    body_a = struct('idx',[],'name','');
    body_b = struct('idx',[],'name','');
  end
  methods
    function obj = RelativeGazeTargetConstraint(robot,tspan,body_a_idx, ...
        body_b_idx,axis,target,gaze_origin,threshold,conethreshold)
      obj = obj@GazeTargetConstraint(robot,tspan,axis,target,gaze_origin,threshold,conethreshold);
      obj.body_a.idx = body_a_idx;
      obj.body_a.name = getLinkName(obj.robot, obj.body_a.idx);
      obj.body_b.idx = body_b_idx;
      obj.body_b.name = getLinkName(obj.robot, obj.body_b.idx);
      obj.num_constraint = 1;
    end

    function [c,dc] = getOrientationError(obj,kinsol)
      u_ga = kinsol.T{obj.body_a.idx}(1:3,1:3)*obj.axis;
      du_ga_dq = reshape(kinsol.dTdq{obj.body_a.idx}(:,1:3)*obj.axis,[obj.robot.getNumDOF,3])';
      [r_go,dr_go_dq] = forwardKin(obj.robot,kinsol,obj.body_a.idx,obj.gaze_origin,0);
      [r_gt,dr_gt_dq] = forwardKin(obj.robot,kinsol,obj.body_b.idx,obj.target,0);
      r_gv = r_gt - r_go;
      dr_gv_dq = dr_gt_dq - dr_go_dq;
      [u_gv,du_gv] = normalizeVec(r_gv);
      du_gv_dq = du_gv*dr_gv_dq;
      c = dot(u_ga,u_gv) - 1;
      dc = u_gv'*du_ga_dq + u_ga'*du_gv_dq;
    end

    function [lb,ub] = getConstraintBnds(obj,t)
      [lb,ub] = getConstraintBnds@GazeTargetConstraint(obj,t);
      lb(2) = [];
      ub(2) = [];
    end
    
    function name = getConstraintName(obj,t)
      if(t>=obj.tspan(1)&&t<=obj.tspan(end))||isempty(t)
        name = {sprintf('%s relative to %s conic gaze constraint at time %10.4f', ...
                        obj.body_a.name,obj.body_b.name,t)};
      else
        name = [];
      end
    end

    function ptr = constructPtr(obj);
      ptr = [];
    end

    function drawConstraint(obj,q,lcmgl)
      norm_axis = obj.axis/norm(obj.axis);

      kinsol = doKinematics(obj.robot,q,false,false);
      wTa = kinsol.T{obj.body_a.idx};
      wTb = kinsol.T{obj.body_b.idx};
      wPa = wTa(1:3,4);
      ang_ax_a = rotmat2axis(wTa(1:3,1:3));
      ang_ax_b = rotmat2axis(wTb(1:3,1:3));
      r_gaze_origin = forwardKin(obj.robot,kinsol,obj.body_a.idx,obj.gaze_origin,0);
      r_target = forwardKin(obj.robot,kinsol,obj.body_b.idx,obj.target,0);

      ang_ax_cone = quat2axis(quatTransform([0;0;1],norm_axis));

      % Draw in Frame B
      bot_lcmgl_translated(lcmgl,wTb(1,4),wTb(2,4),wTb(3,4));
      bot_lcmgl_rotated(lcmgl,ang_ax_b(4)*180/pi,ang_ax_b(1),ang_ax_b(2),ang_ax_b(3));
      bot_lcmgl_draw_axes(lcmgl);

      % Draw target
      bot_lcmgl_color3f(lcmgl, 1, 0, 0);
      bot_lcmgl_sphere(lcmgl,obj.target,0.05,24,24);

      % Rotate and translate back to world frame
      bot_lcmgl_rotated(lcmgl,-ang_ax_b(4)*180/pi,ang_ax_b(1),ang_ax_b(2),ang_ax_b(3));
      bot_lcmgl_translated(lcmgl,-wTb(1,4),-wTb(2,4),-wTb(3,4));

      % Draw line: gaze origin --> target
      bot_lcmgl_begin(lcmgl, lcmgl.LCMGL_LINES);
      bot_lcmgl_color3f(lcmgl, 0, 1, 1);
      bot_lcmgl_vertex3f(lcmgl, r_gaze_origin(1), r_gaze_origin(2), r_gaze_origin(3));
      bot_lcmgl_vertex3f(lcmgl, r_target(1), r_target(2), r_target(3));
      bot_lcmgl_end(lcmgl);

      % Draw in Frame A
      bot_lcmgl_translated(lcmgl,wPa(1),wPa(2),wPa(3));
      bot_lcmgl_rotated(lcmgl,ang_ax_a(4)*180/pi,ang_ax_a(1),ang_ax_a(2),ang_ax_a(3));
      bot_lcmgl_draw_axes(lcmgl);


      %% Draw cone
      bot_lcmgl_translated(lcmgl,obj.gaze_origin(1),obj.gaze_origin(2),obj.gaze_origin(3));
      bot_lcmgl_rotated(lcmgl,ang_ax_cone(4)*180/pi,ang_ax_cone(1),ang_ax_cone(2),ang_ax_cone(3));
      bot_lcmgl_color4f(lcmgl, 0, 1, 0,0.5);
      base_radius = 0.01;
      height = 0.5;
      top_radius = base_radius+height*tan(obj.conethreshold);
      bot_lcmgl_cylinder(lcmgl,[0;0;0],base_radius,top_radius,height,24,24);

      bot_lcmgl_switch_buffer(lcmgl);
    end
  end
end
