classdef RelativePositionConstraint < PositionConstraint
  % Constraining points in body A to be within a bounding box in B' frame on body B
  
  properties(SetAccess = protected)
    body_a = struct('idx',[],'name','');
    body_b = struct('idx',[],'name','');
    bTbp
  end
  
  properties(SetAccess = protected,GetAccess = protected)
    bpTb
  end
  methods(Access = protected)
    function [pos,J] = evalPositions(obj,kinsol)
      % nq = obj.robot.getNumPositions();
      %[pts_world, J_world] = forwardKin(obj.robot,kinsol,obj.body,obj.pts,0);
      [bodyA_pos,JA] = forwardKin(obj.robot,kinsol,obj.body_a.idx,obj.pts,0);
      [wTb,dwTb] = forwardKin(obj.robot,kinsol,obj.body_b.idx,[0;0;0],2);
      [bTw_quat,dbTw_quat] = quatConjugate(wTb(4:7));
      dbTw_quat = dbTw_quat*dwTb(4:7,:);
      [bTw_trans,dbTw_trans] = quatRotateVec(bTw_quat,wTb(1:3));
      bTw_trans = -bTw_trans;
      dbTw_trans = -dbTw_trans*[dbTw_quat;dwTb(1:3,:)];

      [bpTw_trans1,dbpTw_trans1] = quatRotateVec(obj.bpTb(4:7),bTw_trans);
      dbpTw_trans1 = dbpTw_trans1(:,5:7)*dbTw_trans;
      bpTw_trans = bpTw_trans1+obj.bpTb(1:3);
      dbpTw_trans = dbpTw_trans1;
      [bpTw_quat,dbpTw_quat] = quatProduct(obj.bpTb(4:7),bTw_quat);
      dbpTw_quat = dbpTw_quat(:,5:8)*dbTw_quat;

      pos = zeros(3,obj.n_pts);
      J = zeros(3*obj.n_pts,obj.robot.getNumPositions());
      for i = 1:obj.n_pts
        [bp_bodyA_pos1,dbp_bodyA_pos1] = quatRotateVec(bpTw_quat,bodyA_pos(:,i));
        dbp_bodyA_pos1 = dbp_bodyA_pos1*[dbpTw_quat;JA(3*(i-1)+(1:3),:)];
        pos(:,i) = bp_bodyA_pos1+bpTw_trans;
        J(3*(i-1)+(1:3),:) = dbp_bodyA_pos1+dbpTw_trans;
      end
    end
    
    function cnst_names = evalNames(obj,t)
      cnst_names = cell(3*obj.n_pts,1);
      if(isempty(t))
        time_str = '';
      else
        time_str = sprintf('at time %5.2f',t);
      end
      for i = 1:obj.n_pts
        cnst_names{3*(i-1)+1} = sprintf('%s pts(:,%d) in %s x %s',obj.body_a.name,i,obj.body_b.name,time_str);
        cnst_names{3*(i-1)+2} = sprintf('%s pts(:,%d) in %s y %s',obj.body_a.name,i,obj.body_b.name,time_str);
        cnst_names{3*(i-1)+3} = sprintf('%s pts(:,%d) in %s z %s',obj.body_a.name,i,obj.body_b.name,time_str);
      end
    end
  end
  
  methods
function obj = RelativePositionConstraint(robot,pts,lb,ub, ...
                      body_a,body_b,bTbp,tspan)
    % @param robot
      % @param body_a         -- Either:
      %                             * An integer body index or frame id OR
      %                             * A string containing a body or frame name
      % @param body_b         -- Either:
      %                             * An integer body index or frame id OR
      %                             * A string containing a body or frame name
    % @param pts -- A 3xnpts double matrix, points in body A frame
    % @param lb -- A 3xnpts double matrix, the lower bound of the
    % position
    % @param ub -- A 3xnpts double matrix, the upper bound of the
    % position
    % @param bTbp -- A 7x1 vector representating the transformation from frame B' to the body B.
    % bTbp = [bTbp_translation;bTbp_quaternion]
    % bTbp_translation if the vector from
    % origin of body B to B'.
    % bTbp_quaternion is the quaternion
    % representation of rotation.
      if(nargin < 7)
        bTbp = [0;0;0;1;0;0;0];
      end
      if(nargin < 8)
        tspan = [-inf,inf];
      end
      body_a_idx = robot.parseBodyOrFrameID(body_a);
      body_b_idx = robot.parseBodyOrFrameID(body_b);
      ptr = constructPtrRigidBodyConstraintmex(RigidBodyConstraint.RelativePositionConstraintType,...
        robot.getMexModelPtr,pts,lb,ub,body_a_idx,body_b_idx,bTbp,tspan);
      typecheck(bTbp,'double');
      sizecheck(bTbp,[7,1]);
      quat_norm = norm(bTbp(4:7));
      if(abs(quat_norm-1)>1e-5)
        error('Drake:RelativePositionConstraint:bTbp(4;7) is not a quaternion');
      end
      bTbp(4:7) = bTbp(4:7)/quat_norm;

      obj = obj@PositionConstraint(robot, pts, lb, ub,tspan);
      obj.body_a.idx = body_a_idx;
      obj.body_a.name = getBodyOrFrameName(obj.robot, obj.body_a.idx);
      obj.body_b.idx = body_b_idx;
      obj.body_b.name = getBodyOrFrameName(obj.robot, obj.body_b.idx);
      obj.bTbp = bTbp;
      bpTb_quat = quatConjugate(bTbp(4:7));
      obj.bpTb = [-quatRotateVec(bpTb_quat,bTbp(1:3));bpTb_quat];
      obj.mex_ptr = ptr;
      obj.type = RigidBodyConstraint.RelativePositionConstraintType;
    end

    function drawConstraint(obj,q,lcmgl)
      kinsol = doKinematics(obj.robot,q,false,false);
      pts_w = forwardKin(obj.robot,kinsol,1,obj.pts);
      wTbp = kinsol.T{obj.body_b.idx}*invHT([quat2rotmat(obj.bpTb(4:7)) obj.bpTb(1:3);0 0 0 1]);
      wPbp = wTbp(1:3,4);
      lcmgl.glDrawAxes();
      for pt = pts_w
        lcmgl.glColor3f(0.25,0.25,0.25);
        lcmgl.sphere(pt, 0.02, 36, 36);
      end
      a = rotmat2axis(wTbp(1:3,1:3));
      lcmgl.glTranslated(wPbp(1),wPbp(2),wPbp(3));
      lcmgl.glRotated(a(4)*180/pi,a(1),a(2),a(3));
      lcmgl.glDrawAxes();
      lcmgl.glColor4f(0,1,0,0.5);
      lcmgl.box((obj.lb+obj.ub)/2,obj.ub-obj.lb);
      lcmgl.switchBuffers();
    end
    
    function joint_idx = kinematicsPathJoints(obj)
      [~,joint_path] = obj.robot.findKinematicPath(obj.body_a.idx,obj.body_b.idx);
      joint_idx = vertcat(obj.robot.body(joint_path).position_num)';
    end
  end
end
