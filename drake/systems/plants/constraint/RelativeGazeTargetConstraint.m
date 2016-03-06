classdef RelativeGazeTargetConstraint < GazeTargetConstraint
  % This constrains that a camera on body_a can gaze at a point on body_b
  % @param robot
  % @param body_a             -- A scalar, the index of body_a
  % @param body_b             -- A scalar, the index of body_b
  % @param axis                   -- A 3x1 unit vector, the gaze axis on body_a frame
  % @param target                 -- A 3x1 vector, the gaze target on body_b frame
  % @param gaze_origin            -- A 3x1 vector, the gaze origin on body_a frame
  % @param conethreshold          -- A double scalar, the angle of the gaze cone, default
  %                                  is 0
  % @param tspan                  -- A 1x2 vector, the time span of the constraint,
  %                                  default is [-inf inf]
  properties(SetAccess = protected)
    body_a = struct('idx',[],'name','');
    body_b = struct('idx',[],'name','');
  end
  
  methods(Access = protected)
    function [c,dc] = evalValidTime(obj,kinsol)
      [pos_a,dpos_a_dq] = obj.robot.forwardKin(kinsol,obj.body_a.idx,[0;0;0],2);
      [u_ga,du_ga] = quatRotateVec(pos_a(4:7),obj.axis);
      du_ga_dq = du_ga(:,1:4)*dpos_a_dq(4:7,:);
%       u_ga = kinsol.T{obj.body_a.idx}(1:3,1:3)*obj.axis;
%       du_ga_dq = reshape(kinsol.dTdq{obj.body_a.idx}(:,1:3)*obj.axis,[obj.robot.getNumPositions,3])';
      [r_go,dr_go_dq] = forwardKin(obj.robot,kinsol,obj.body_a.idx,obj.gaze_origin,0);
      [r_gt,dr_gt_dq] = forwardKin(obj.robot,kinsol,obj.body_b.idx,obj.target,0);
      r_gv = r_gt - r_go;
      dr_gv_dq = dr_gt_dq - dr_go_dq;
      [u_gv,du_gv] = normalizeVec(r_gv);
      du_gv_dq = du_gv*dr_gv_dq;
      c = dot(u_ga,u_gv) - 1;
      dc = u_gv'*du_ga_dq + u_ga'*du_gv_dq;
    end
  end
  
  methods
    function obj = RelativeGazeTargetConstraint(robot,body_a, ...
        body_b,axis,target,gaze_origin,conethreshold,tspan)
      if(nargin < 8)
        tspan = [-inf inf];
      end
      if(nargin<7)
        conethreshold = 0;
      end
      body_a_idx = robot.parseBodyOrFrameID(body_a);
      body_b_idx = robot.parseBodyOrFrameID(body_b);
      ptr = constructPtrRigidBodyConstraintmex(RigidBodyConstraint.RelativeGazeTargetConstraintType,robot.getMexModelPtr,body_a_idx,body_b_idx,axis,target,gaze_origin,conethreshold,tspan);
      obj = obj@GazeTargetConstraint(robot,axis,target,gaze_origin,conethreshold,tspan);
      obj.body_a.idx = body_a_idx;
      obj.body_a.name = getBodyOrFrameName(obj.robot, obj.body_a.idx);
      obj.body_b.idx = body_b_idx;
      obj.body_b.name = getBodyOrFrameName(obj.robot, obj.body_b.idx);
      obj.type = RigidBodyConstraint.RelativeGazeTargetConstraintType;
      obj.mex_ptr = ptr;
    end
    
    function name_str = name(obj,t)
      if(obj.isTimeValid(t))
        name_str = {sprintf('%s relative to %s conic gaze constraint at time %10.4f', ...
                        obj.body_a.name,obj.body_b.name,t)};
      else
        name_str = [];
      end
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

      ang_ax_ax = cross([0;0;1],norm_axis);
      ang_ax_ang = acos([0;0;1]'*norm_axis);

      % Draw in Frame B
      lcmgl.glTranslated(wTb(1,4),wTb(2,4),wTb(3,4));
      lcmgl.glRotated(ang_ax_b(4)*180/pi,ang_ax_b(1),ang_ax_b(2),ang_ax_b(3));
      lcmgl.glDrawAxes();

      % Draw target
      lcmgl.glColor3f(1, 0, 0);
      lcmgl.sphere(obj.target,0.05,24,24);

      % Rotate and translate back to world frame
      lcmgl.glRotated(-ang_ax_b(4)*180/pi,ang_ax_b(1),ang_ax_b(2),ang_ax_b(3));
      lcmgl.glTranslated(-wTb(1,4),-wTb(2,4),-wTb(3,4));

      % Draw line: gaze origin --> target
      lcmgl.glBegin( lcmgl.LCMGL_LINES);
      lcmgl.glColor3f( 0, 1, 1);
      lcmgl.glVertex3f(r_gaze_origin(1), r_gaze_origin(2), r_gaze_origin(3));
      lcmgl.glVertex3f( r_target(1), r_target(2), r_target(3));
      lcmgl.glEnd();

      % Draw in Frame A
      lcmgl.glTranslated(wPa(1),wPa(2),wPa(3));
      lcmgl.glRotated(ang_ax_a(4)*180/pi,ang_ax_a(1),ang_ax_a(2),ang_ax_a(3));
      lcmgl.glDrawAxes();


      %% Draw cone
      lcmgl.glTranslated(obj.gaze_origin(1),obj.gaze_origin(2),obj.gaze_origin(3));
      lcmgl.glRotated(ang_ax_ang*180/pi,ang_ax_ax(1),ang_ax_ax(2),ang_ax_ax(3));
      lcmgl.glColor4f( 0, 1, 0,0.5);
      base_radius = 0.01;
      height = 0.5;
      top_radius = base_radius+height*tan(obj.conethreshold);
      lcmgl.cylinder([0;0;0],base_radius,top_radius,height,24,24);

      lcmgl.switchBuffers();
    end
    
    function joint_idx = kinematicsPathJoints(obj)
      [~,joint_path] = obj.robot.findKinematicPath(obj.body_a.idx,obj.body_b.idx);
      joint_idx = vertcat(obj.robot.body(joint_path).position_num)';
    end
  end
end
