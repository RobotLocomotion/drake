classdef RelativeGazeDirConstraint < GazeDirConstraint
  % This constrains that an axis on body A is aligned with a direction on body
  % B to within a given threshold
  % @param robot
  % @param body_a             -- A scalar, the index of body_a
  % @param body_b             -- A scalar, the index of body_b
  % @param axis                   -- A 3x1 unit vector, in the body_a frame
  % @param dir                    -- A 3x1 unit vector, in the body_b frame
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
      [axis_pos,daxis_pos] = forwardKin(obj.robot,kinsol,obj.body_a.idx,[zeros(3,1) obj.axis],0);
      [dir_pos,ddir_pos] = forwardKin(obj.robot,kinsol,obj.body_b.idx,[zeros(3,1) obj.dir],0);
      axis_world = axis_pos(:,2)-axis_pos(:,1);
      dir_world = dir_pos(:,2)-dir_pos(:,1);
      daxis_world = daxis_pos(4:6,:)-daxis_pos(1:3,:);
      ddir_world = ddir_pos(4:6,:)-ddir_pos(1:3,:);
      c = axis_world'*dir_world-1;
      dc = dir_world'*daxis_world + axis_world'*ddir_world;
    end
  end
  
  methods  
    function obj = RelativeGazeDirConstraint(robot,body_a, ...
                                             body_b,axis,dir, ...
                                             conethreshold,tspan)
      if(nargin < 7)
        tspan = [-inf inf];
      end
      if(nargin<6)
        conethreshold = 0;
      end
      body_a_idx = robot.parseBodyOrFrameID(body_a);
      body_b_idx = robot.parseBodyOrFrameID(body_b);
      ptr = constructPtrRigidBodyConstraintmex(RigidBodyConstraint.RelativeGazeDirConstraintType,robot.getMexModelPtr,body_a_idx,body_b_idx,axis,dir,conethreshold,tspan);
      obj = obj@GazeDirConstraint(robot,axis,dir,conethreshold,tspan);
      obj.body_a.idx = body_a_idx;
      obj.body_a.name = getBodyOrFrameName(obj.robot, obj.body_a.idx);
      obj.body_b.idx = body_b_idx;
      obj.body_b.name = getBodyOrFrameName(obj.robot, obj.body_b.idx);
      obj.type = RigidBodyConstraint.RelativeGazeDirConstraintType;
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
      norm_dir = obj.dir/norm(obj.dir);
      norm_axis = obj.axis/norm(obj.axis);

      kinsol = doKinematics(obj.robot,q,false,false);
      wTa = kinsol.T{obj.body_a.idx};
      wTb = kinsol.T{obj.body_b.idx};
      wPa = wTa(1:3,4);
      ang_ax_a = rotmat2axis(wTa(1:3,1:3));
      ang_ax_b = rotmat2axis(wTb(1:3,1:3));

      ang_dir_ax = cross([0;0;1],norm_dir);
      ang_dir_ang = acos([0;0;1]'*norm_dir);

      % Draw in Frame B
      lcmgl.glTranslated(wTb(1,4),wTb(2,4),wTb(3,4));
      lcmgl.glRotated(ang_ax_b(4)*180/pi,ang_ax_b(1),ang_ax_b(2),ang_ax_b(3));
      lcmgl.glDrawAxes();
      
      % Rotate and translate back to world frame
      lcmgl.glRotated(-ang_ax_b(4)*180/pi,ang_ax_b(1),ang_ax_b(2),ang_ax_b(3));
      lcmgl.glTranslated(-wTb(1,4),-wTb(2,4),-wTb(3,4));

      % Draw cone at origin of Frame A, but with orientation of Frame B
      lcmgl.glTranslated(wPa(1),wPa(2),wPa(3));
      lcmgl.glRotated(ang_ax_b(4)*180/pi,ang_ax_b(1),ang_ax_b(2),ang_ax_b(3));
      lcmgl.glRotated(ang_dir_ang*180/pi,ang_dir_ax(1),ang_dir_ax(2),ang_dir_ax(3));
      lcmgl.glColor4f( 0, 1, 0,0.5);
      base_radius = 0.01;
      height = 0.5;
      top_radius = base_radius+height*tan(obj.conethreshold);
      lcmgl.cylinder([0;0;0],base_radius,top_radius,height,24,24);

      % Rotate and translate back to world frame
      lcmgl.glRotated(-ang_dir_ang*180/pi,ang_dir_ax(1),ang_dir_ax(2),ang_dir_ax(3));
      lcmgl.glRotated(-ang_ax_b(4)*180/pi,ang_ax_b(1),ang_ax_b(2),ang_ax_b(3));
      lcmgl.glTranslated(-wPa(1),-wPa(2),-wPa(3));


      % Draw in Frame A
      lcmgl.glTranslated(wPa(1),wPa(2),wPa(3));
      lcmgl.glRotated(ang_ax_a(4)*180/pi,ang_ax_a(1),ang_ax_a(2),ang_ax_a(3));
      lcmgl.glDrawAxes();


      % Draw gaze axis
      lcmgl.glBegin( lcmgl.LCMGL_LINES);
      lcmgl.glColor3f( 0, 1, 1);
      %lcmgl.glVertex3f(wPa(1),wPa(2),wPa(3));
      %lcmgl.glVertex3f( r_axis(1), r_axis(2), r_axis(3));
      lcmgl.glVertex3f(0, 0, 0);
      lcmgl.glVertex3f( norm_axis(1), norm_axis(2), norm_axis(3));
      lcmgl.glEnd();

      lcmgl.switchBuffers();
    end
    
    function joint_idx = kinematicsPathJoints(obj)
      [~,joint_path] = obj.robot.findKinematicPath(obj.body_a.idx,obj.body_b.idx);
      joint_idx = vertcat(obj.robot.body(joint_path).position_num)';
    end
  end
end
