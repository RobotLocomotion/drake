classdef RelativeGazeOrientConstraint < GazeOrientConstraint
  properties(SetAccess = protected)
    body_a = struct('idx',[],'name','');
    body_b = struct('idx',[],'name','');
  end
  
  methods(Access = protected)
    function [quat, dquat_dq] = evalOrientation(obj,kinsol)
      [x_a,J_a] = forwardKin(obj.robot,kinsol,obj.body_a.idx,[0;0;0],2);
      quat_a = x_a(4:7);
      dquat_a = J_a(4:7,:);
      [x_b,J_b] = forwardKin(obj.robot,kinsol,obj.body_b.idx,[0;0;0],2);
      quat_b = x_b(4:7);
      dquat_b = J_b(4:7,:);
      [quat, dquat] = quatDiff(quat_b,quat_a);
      dquat_dq = dquat*[dquat_b;dquat_a];
    end
  end
  
  methods
    function obj = RelativeGazeOrientConstraint(robot,tspan,body_a, ...
        body_b,axis,quat_des, ...
        threshold,conethreshold)
      obj = obj@GazeOrientConstraint(robot,tspan,axis,quat_des,threshold, ...
        conethreshold);
      body_a_idx = robot.parseBodyOrFrameID(body_a);
      body_b_idx = robot.parseBodyOrFrameID(body_b);
      obj.body_a.idx = body_a_idx;
      obj.body_a.name = getBodyOrFrameName(obj.robot, obj.body_a.idx);
      obj.body_b.idx = body_b_idx;
      obj.body_b.name = getBodyOrFrameName(obj.robot, obj.body_b.idx);
    end

    

    function drawConstraint(obj,q,lcmgl)
      norm_axis = obj.axis/norm(obj.axis);
      axis_perp = cross(norm_axis,[1;0;0]);
      if norm(axis_perp) < eps
        axis_perp = cross(norm_axis,[0;1;0]);
      end

      axis_perp_norm = 0.1*axis_perp/norm(axis_perp);

      kinsol = doKinematics(obj.robot,q,false,false);
      wTa = kinsol.T{obj.body_a.idx};
      wTb = kinsol.T{obj.body_b.idx};
      ang_ax_a = rotmat2axis(wTa(1:3,1:3));
      ang_ax_b = rotmat2axis(wTb(1:3,1:3));
      ang_ax_des = quat2axis(quatConjugate(obj.quat_des));
      ang_ax_des(4) = -ang_ax_des(4);
      ang_ax_cone = quat2axis(quatTransform([0;0;1],norm_axis));

      % Draw in Frame B
      bot_lcmgl_translated(lcmgl,wTb(1,4),wTb(2,4),wTb(3,4));
      bot_lcmgl_rotated(lcmgl,ang_ax_b(4)*180/pi,ang_ax_b(1),ang_ax_b(2),ang_ax_b(3));
      bot_lcmgl_draw_axes(lcmgl);

      % Draw in desired frame
      bot_lcmgl_rotated(lcmgl,ang_ax_des(4)*180/pi,ang_ax_des(1),ang_ax_des(2),ang_ax_des(3));

      bot_lcmgl_line_width(lcmgl, 10);

      bot_lcmgl_begin(lcmgl, lcmgl.LCMGL_LINES);
      bot_lcmgl_color3f(lcmgl, 0, 1, 1);
      bot_lcmgl_vertex3f(lcmgl, 0, 0, 0);
      bot_lcmgl_vertex3f(lcmgl, norm_axis(1), norm_axis(2), norm_axis(3));
      bot_lcmgl_end(lcmgl);

      bot_lcmgl_begin(lcmgl, lcmgl.LCMGL_LINES);
      bot_lcmgl_color3f(lcmgl, 0, 1, 1);
      bot_lcmgl_vertex3f(lcmgl, 0, 0, 0);
      bot_lcmgl_vertex3f(lcmgl, axis_perp_norm(1), axis_perp_norm(2), axis_perp_norm(3));
      bot_lcmgl_end(lcmgl);

      bot_lcmgl_line_width(lcmgl, 1);

      % Rotate and translate back to world frame
      bot_lcmgl_rotated(lcmgl,-ang_ax_des(4)*180/pi,ang_ax_des(1),ang_ax_des(2),ang_ax_des(3));
      bot_lcmgl_rotated(lcmgl,-ang_ax_b(4)*180/pi,ang_ax_b(1),ang_ax_b(2),ang_ax_b(3));
      bot_lcmgl_translated(lcmgl,-wTb(1,4),-wTb(2,4),-wTb(3,4));

      % Draw in Frame A
      bot_lcmgl_translated(lcmgl,wTa(1,4),wTa(2,4),wTa(3,4));
      bot_lcmgl_rotated(lcmgl,ang_ax_a(4)*180/pi,ang_ax_a(1),ang_ax_a(2),ang_ax_a(3));

      bot_lcmgl_draw_axes(lcmgl);

      bot_lcmgl_line_width(lcmgl, 10);

      %bot_lcmgl_begin(lcmgl, lcmgl.LCMGL_LINES);
      %bot_lcmgl_color3f(lcmgl, 1, 1, 0);
      %bot_lcmgl_vertex3f(lcmgl, 0, 0, 0);
      %bot_lcmgl_vertex3f(lcmgl, norm_axis(1), norm_axis(2), norm_axis(3));
      %bot_lcmgl_end(lcmgl);

      bot_lcmgl_begin(lcmgl, lcmgl.LCMGL_LINES);
      bot_lcmgl_color3f(lcmgl, 1, 1, 0);
      bot_lcmgl_vertex3f(lcmgl, 0, 0, 0);
      bot_lcmgl_vertex3f(lcmgl, axis_perp_norm(1), axis_perp_norm(2), axis_perp_norm(3));
      bot_lcmgl_end(lcmgl);

      bot_lcmgl_line_width(lcmgl, 1);


      % Rotate to orientation of desired frame (still at origin of Frame A)
      bot_lcmgl_rotated(lcmgl,-ang_ax_a(4)*180/pi,ang_ax_a(1),ang_ax_a(2),ang_ax_a(3));
      bot_lcmgl_rotated(lcmgl,ang_ax_b(4)*180/pi,ang_ax_b(1),ang_ax_b(2),ang_ax_b(3));
      bot_lcmgl_rotated(lcmgl,ang_ax_des(4)*180/pi,ang_ax_des(1),ang_ax_des(2),ang_ax_des(3));

      bot_lcmgl_line_width(lcmgl, 5);

      bot_lcmgl_begin(lcmgl, lcmgl.LCMGL_LINES);
      bot_lcmgl_color3f(lcmgl, 0, 1, 1);
      bot_lcmgl_vertex3f(lcmgl, 0, 0, 0);
      bot_lcmgl_vertex3f(lcmgl, norm_axis(1), norm_axis(2), norm_axis(3));
      bot_lcmgl_end(lcmgl);

      bot_lcmgl_begin(lcmgl, lcmgl.LCMGL_LINES);
      bot_lcmgl_color3f(lcmgl, 0, 1, 1);
      bot_lcmgl_vertex3f(lcmgl, 0, 0, 0);
      bot_lcmgl_vertex3f(lcmgl, axis_perp_norm(1), axis_perp_norm(2), axis_perp_norm(3));
      bot_lcmgl_end(lcmgl);

      bot_lcmgl_line_width(lcmgl, 1);

      % Draw cone
      bot_lcmgl_rotated(lcmgl,-ang_ax_des(4)*180/pi,ang_ax_des(1),ang_ax_des(2),ang_ax_des(3));
      bot_lcmgl_rotated(lcmgl,-ang_ax_b(4)*180/pi,ang_ax_b(1),ang_ax_b(2),ang_ax_b(3));
      bot_lcmgl_rotated(lcmgl,ang_ax_a(4)*180/pi,ang_ax_a(1),ang_ax_a(2),ang_ax_a(3));
      bot_lcmgl_rotated(lcmgl,ang_ax_cone(4)*180/pi,ang_ax_cone(1),ang_ax_cone(2),ang_ax_cone(3));
      bot_lcmgl_color4f(lcmgl, 0, 1, 0,0.5);
      base_radius = 0.01;
      height = 0.5;
      top_radius = base_radius+height*tan(obj.conethreshold);
      bot_lcmgl_cylinder(lcmgl,[0;0;0],base_radius,top_radius,height,24,24);
      bot_lcmgl_line_width(lcmgl, 10);

      bot_lcmgl_switch_buffer(lcmgl);
    end

    function name_str = name(obj,t)
      if(t>=obj.tspan(1)&&t<=obj.tspan(end))||isempty(t)
        name_str = {sprintf('%s relative to %s conic gaze constraint at time %10.4f', ...
                        obj.body_a.name,obj.body_b.name,t);...
                sprintf('%s relative to %s conic gaze constraint at time %10.4f', ...
                        obj.body_a.name,obj.body_b.name,t)};
      else
        name_str = [];
      end
    end

  end
end
