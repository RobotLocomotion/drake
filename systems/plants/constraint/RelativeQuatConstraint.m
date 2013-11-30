classdef RelativeQuatConstraint < QuatConstraint
  % Constrains the orientation of the body reference frame of link A relative
  % to Frame B' fixed to Body B. The quaternions representing the orientations
  % of Frame A and Frame B' must satisfy:
  %
  % (quat_a'*quat_bp)^2 in [1-tol, 1]
  properties(SetAccess = protected)
    bodyA = struct('idx',[],'name','');
    bodyB = struct('idx',[],'name','','q_bp2b',[]);
  end
  
  methods(Access = protected)
    function [orient_prod, dorient_prod] = evalOrientationProduct(obj,kinsol)
      [pos_a,J_a] = forwardKin(obj.robot,kinsol,obj.bodyA.idx,[0;0;0],2);
      [pos_b,J_b] = forwardKin(obj.robot,kinsol,obj.bodyB.idx,[0;0;0],2);
      q_a2w = pos_a(4:7,1);
      dq_a2w = J_a(4:7,:);
      q_b2w = pos_b(4:7,1);
      dq_b2w = J_b(4:7,:);

      [q_bp2w,dq_bp2w] = quatProduct(q_b2w,obj.bodyB.q_bp2b,dq_b2w,0*dq_b2w);

      orient_prod = q_a2w'*q_bp2w;
      dorient_prod = q_bp2w'*dq_a2w + q_a2w'*dq_bp2w;
    end
  end
  
  methods
    function obj = RelativeQuatConstraint(robot,bodyA_idx,bodyB_idx,trans_bp2b,tol,tspan)
      typecheck(bodyA_idx,'double');
      typecheck(bodyB_idx,'double');
      sizecheck(trans_bp2b,[4,1]);

      obj = obj@QuatConstraint(robot, tol,tspan);
      obj.bodyA.idx = bodyA_idx;
      obj.bodyA.name = getLinkName(obj.robot, obj.bodyA.idx);
      obj.bodyB.idx = bodyB_idx;
      obj.bodyB.name = getLinkName(obj.robot, obj.bodyB.idx);
      obj.bodyB.q_bp2b = trans_bp2b./norm(trans_bp2b);
    end

    

    function name_str = name(obj,t)
      if(t>=obj.tspan(1)&&t<=obj.tspan(end))||isempty(t)
        name_str = {sprintf('%s relative to %s orientation constraint at time %10.4f', ...
                        obj.bodyA.name,obj.bodyB.name,t)};
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

