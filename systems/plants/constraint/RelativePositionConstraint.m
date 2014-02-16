classdef RelativePositionConstraint < PositionConstraint
  % Constraining points in bodyA to be within a bounding box in B' frame on bodyB
  % @param robot
  % @param bodyA_idx             -- An int scalar, the bodyA index
  % @param bodyB_idx             -- An int scalar, the bodyB index
  % @param pts                   -- A 3xnpts double matrix, points in bodyA frame
  % @param lb                    -- A 3xnpts double matrix, the lower bound of the
  % position
  % @param ub                    -- A 3xnpts double matrix, the upper bound of the
  % position
  % @param bTbp                  -- A 4x4 homogeneous transformation matrix, the
  % transformation from B' frame to bodyB frame
  properties(SetAccess = protected)
    bodyA_idx
    bodyB_idx
    bodyA_name
    bodyB_name
    bodyB_bpTb
  end
  
  methods(Access = protected)
    function [pos,J] = evalPositions(obj,kinsol)
      if kinsol.mex
        error('The mex version of RelativePositionConstraint is not yet implemented');
      else
        nq = obj.robot.num_q;
        %[pts_world, J_world] = forwardKin(obj.robot,kinsol,obj.body,obj.pts,0);
        pts = [obj.pts;ones(1,obj.n_pts)];

        [bTw,d_bTw_dq_ht] = invHT(kinsol.T{obj.bodyB_idx}, ...
          kinsol.dTdq{obj.bodyB_idx});
        wTa = kinsol.T{obj.bodyA_idx};
        d_wTa_dq_ht = kinsol.dTdq{obj.bodyA_idx};

        d_bTa_dq_std = matGradMultMat(bTw, wTa, ...
          jacHt2Std(d_bTw_dq_ht), jacHt2Std(d_wTa_dq_ht));

        pos = obj.bodyB_bpTb*bTw*wTa*pts;
        J  = reshape(obj.bodyB_bpTb*reshape(matGradMult(d_bTa_dq_std, pts),4,nq*obj.n_pts),4*obj.n_pts,nq);

        pos = pos(1:3,:);
        J(4:4:end,:) = [];
      end
    end
  end
  
  methods
    function obj = RelativePositionConstraint(robot,pts,lb,ub, ...
                      bodyA_idx,bodyB_idx,bTbp,tspan)
      typecheck(bodyA_idx,'double');
      typecheck(bodyB_idx,'double');
      isHT(bTbp);

      obj = obj@PositionConstraint(robot, pts, lb, ub,tspan);
      obj.bodyA_idx = bodyA_idx;
      obj.bodyA_name = getLinkName(obj.robot, obj.bodyA_idx);
      obj.bodyB_idx = bodyB_idx;
      obj.bodyB_name = getLinkName(obj.robot, obj.bodyB_idx);
      obj.bodyB_bpTb = invHT(bTbp);
    end

    

    function name_str = name(obj,t)
      if(t>=obj.tspan(1)&&t<=obj.tspan(end))||isempty(t)
        name_str = repmat({sprintf('%s relative to %s position constraint at time %10.4f', ...
                                obj.bodyA_name,obj.bodyB_name,t)},obj.getNumConstraint(),1);
      else
        name_str = [];
      end
    end


    function drawConstraint(obj,q,lcmgl)
      kinsol = doKinematics(obj.robot,q,false,false);
      pts_w = forwardKin(obj.robot,kinsol,1,obj.pts);
      wTbp = kinsol.T{obj.bodyB_idx}*invHT(obj.bodyB_bpTb);
      wPbp = wTbp(1:3,4);
      bot_lcmgl_draw_axes(lcmgl);
      for pt = pts_w
        bot_lcmgl_color3f(lcmgl,0.25,0.25,0.25);
        bot_lcmgl_sphere(lcmgl, pt, 0.02, 36, 36);
      end
      a = rotmat2axis(wTbp(1:3,1:3));
      bot_lcmgl_translated(lcmgl,wPbp(1),wPbp(2),wPbp(3));
      bot_lcmgl_rotated(lcmgl,a(4)*180/pi,a(1),a(2),a(3));
      bot_lcmgl_draw_axes(lcmgl);
      bot_lcmgl_color4f(lcmgl,0,1,0,0.5);
      bot_lcmgl_box(lcmgl,(obj.lb+obj.ub)/2,obj.ub-obj.lb);
      bot_lcmgl_switch_buffer(lcmgl);
    end
  end
end
