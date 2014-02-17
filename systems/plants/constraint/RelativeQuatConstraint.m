classdef RelativeQuatConstraint < QuatConstraint
  % Constrains the orientation of the body reference frame of link A relative
  % to the frame of Body B. The quaternions representing the orientations
  % of Frame A and Frame B must satisfy:
  %
  % 2(quat_a2b'*quat_des)^2-1 in [cos(tol) 1]
  properties(SetAccess = protected)
    bodyA_idx
    bodyB_idx
    bodyA_name
    bodyB_name
  end
  
  methods(Access = protected)
    function [orient_prod, dorient_prod] = evalOrientationProduct(obj,kinsol)
      [pos_a,J_a] = forwardKin(obj.robot,kinsol,obj.bodyA.idx,[0;0;0],2);
      [pos_b,J_b] = forwardKin(obj.robot,kinsol,obj.bodyB.idx,[0;0;0],2);
      quat_a2w = pos_a(4:7,1);
      dquat_a2w = J_a(4:7,:);
      quat_b2w = pos_b(4:7,1);
      dquat_b2w = J_b(4:7,:);
      [quat_w2b,dquat_w2b] = quatConjugate(quat_b2w);
      dquat_w2b = dquat_w2b*dquat_b2w;

      [quat_a2b,dquat_a2b] = quatProduct(quat_w2b,quat_a2w);
      dquat_a2bdq = dquat_a2b*[dquat_w2b;dquat_a2w];

      orient_prod = 2*(quat_a2b'*obj.quat_des)^2-1;
      dorient_prod = 4*(quat_a2b'*obj.quat_des)*obj.quat_des'*dquat_a2bdq;
    end
  end
  
  methods
    function obj = RelativeQuatConstraint(robot,bodyA_idx,bodyB_idx,quat_des,tol,tspan)
      % @param robot
      % @param bodyA_idx     -- An int scalar, the bodyA index
      % @param bodyB_idx     -- An int scalar, the bodyB index
      % @param quat_des      -- A 4x1 vector. The quaternion that represents the desired rotation
      % from body A frame to bodyB frame
      % @param tol           -- A scalar, the maximum allowable angle between the desired
      % orientation and the actual orientation
      if(nargin<6)
        tspan = [-inf,inf];
      end
      obj = obj@QuatConstraint(robot, tol,tspan);
      typecheck(bodyA_idx,'double');
      sizecheck(bodyA_idx,[1,1]);
      obj.bodyA_idx = bodyA_idx;
      obj.bodyA_name = obj.robot.getBody(obj.bodyA_idx).linkname;
      typecheck(bodyB_idx,'double');
      sizecheck(bodyB_idx,[1,1]);
      obj.bodyB_idx = bodyB_idx;
      obj.bodyB_name = obj.robot.getBody(obj.bodyB_idx).linkname;
      typecheck(quat_des,'double');
      sizecheck(quat_des,[4,1]);
      quat_norm = norm(quat_des);
      if(abs(quat_norm-1)>1e-5)
        error('Drake:RelativeQuatConstraint:quat_des should be a unit vector');
      end
      obj.quat_des = quat_des/quat_norm;
      obj.type = RigidBodyConstraint.RelativeQuatConstraintType;
    end

    

    function name_str = name(obj,t)
      if(obj.isTimeValid(t))
        if(isempty(t))
          time_str = '';
        else
          time_str = sprintf('at time %5.2f',t);
        end
          
        name_str = {sprintf('%s relative to %s orientation constraint %s', ...
                        obj.bodyA.name,obj.bodyB.name,time_str)};
      else
        name_str = [];
      end
    end


    
    function drawConstraint(obj,q,lcmgl)
      kinsol = doKinematics(obj.robot,q,false,false);
      wTa = kinsol.T{obj.bodyA.idx};
      wTb = kinsol.T{obj.bodyB.idx};
      bTbp = [quat2rotmat(obj.bodyB.q_bp2b),zeros(3,1); zeros(1,3),1];
      wTbp = wTb*bTbp;
      ang_ax_a = rotmat2axis(wTa(1:3,1:3));
      ang_ax_bp = rotmat2axis(wTbp(1:3,1:3));

      bot_lcmgl_translated(lcmgl,wTa(1,4),wTa(2,4),wTa(3,4));
      bot_lcmgl_rotated(lcmgl,ang_ax_a(4)*180/pi,ang_ax_a(1),ang_ax_a(2),ang_ax_a(3));
      bot_lcmgl_draw_axes(lcmgl);

      bot_lcmgl_rotated(lcmgl,-ang_ax_a(4)*180/pi,ang_ax_a(1),ang_ax_a(2),ang_ax_a(3));
      bot_lcmgl_translated(lcmgl,-wTa(1,4),-wTa(2,4),-wTa(3,4));

      bot_lcmgl_translated(lcmgl,wTbp(1,4),wTbp(2,4),wTbp(3,4));
      bot_lcmgl_rotated(lcmgl,ang_ax_bp(4)*180/pi,ang_ax_bp(1),ang_ax_bp(2),ang_ax_bp(3));
      bot_lcmgl_draw_axes(lcmgl);

      bot_lcmgl_switch_buffer(lcmgl);
    end
  end
end

