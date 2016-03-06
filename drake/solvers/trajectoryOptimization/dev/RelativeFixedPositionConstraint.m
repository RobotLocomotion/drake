classdef RelativeFixedPositionConstraint < Constraint
  % constrain the position of point body_pts_A on body_A to be fixed in a
  % given body body_B
  properties(SetAccess = protected)
    robot
    body_A
    body_pts_A
    body_B
    body_A_name;
    body_B_name;
    num_postures
    num_pts;
  end
  
  methods
    function obj = RelativeFixedPositionConstraint(robot,body_A,body_pts_A,body_B,num_postures)
      if(~isa(robot,'RigidBodyManipulator') && ~isa(robot,'TimeSteppingRigidBodyManipulator'))
        error('Drake:RelativeFixedPositionConstraint:robot should be a RigidBodyManipulator or a TimeSteppingRigidBodyManipulator');
      end
      nq = robot.getNumPositions();
      body_A_idx = robot.parseBodyOrFrameID(body_A);
      body_B_idx = robot.parseBodyOrFrameID(body_B);
      pts_size = size(body_pts_A);
      if(~isnumeric(body_pts_A) || length(pts_size) ~= 2 || pts_size(1) ~= 3)
        error('Drake:RelativeFixedPositionConstraint: body_pts_A should be a 3 x num_pts double matrix');
      end
      m_num_pts = pts_size(2);
      if(~isnumeric(num_postures) || numel(num_postures) ~= 1 || num_postures<2)
        error('Drake:RelativeFixedPositionConstraint: num_postures should be a integer no smaller than 2');
      end
      num_postures = floor(num_postures);
      obj = obj@Constraint(zeros(m_num_pts,1),zeros(m_num_pts,1),nq*num_postures,1);
      obj.robot = robot; 
      obj.body_A = body_A_idx;
      obj.body_B = body_B_idx;
      obj.body_pts_A = body_pts_A;
      obj.body_A_name = getBodyOrFrameName(obj.robot,obj.body_A);
      obj.body_B_name = getBodyOrFrameName(obj.robot,obj.body_B);
      obj.num_postures = num_postures;
      obj.num_pts = m_num_pts;
      names = cell(obj.num_pts,1);
      for i = 1:obj.num_pts
        names{i} = sprintf('body %s pts(:,%d) fixed in body %s',obj.body_A_name,i,obj.body_B_name);
      end
      obj = obj.setName(names);
    end
  end
  
  methods(Access = protected)
    function [c,dc] = constraintEval(obj,varargin)
      % [c,dc] =
      % constraintEval(obj,q,kinsol1,kinsol2,...,kinsol_num_postures)
      if(nargin ~= 2+obj.num_postures)
        error('Drake:RelativeFixedPositionConstraint:constraintEval: The input should be constraintEval(obj,q,kinsol1, kinsol2,...,kinsol_num_postures)');
      end
      kinsol = varargin(2:end);
      [pos1,JA1] = forwardKin(obj.robot,kinsol{1},obj.body_A,obj.body_pts_A,0);
      [pts1_in_B,P1,JB1] = bodyKin(obj.robot,kinsol{1},obj.body_B,pos1);
      J1 = JB1+P1*JA1;
      c = zeros(obj.num_pts,1);
      nq = obj.robot.getNumPositions();
      dc = zeros(obj.num_pts,obj.num_postures*nq);
      for i = 2:obj.num_postures
        [pos2,JA2] = forwardKin(obj.robot,kinsol{i},obj.body_A,obj.body_pts_A,0);
        [pts2_in_B,P2,JB2] = bodyKin(obj.robot,kinsol{i},obj.body_B,pos2);
        pos_diff = pts2_in_B-pts1_in_B;
        c = c+sum(pos_diff.^2,1)';
        J2 = JB2+P2*JA2;
        dcdpos_diff = sparse(reshape(bsxfun(@times,ones(3,1),1:obj.num_pts),[],1),(1:3*obj.num_pts)',2*pos_diff(:),obj.num_pts,3*obj.num_pts);
        dc(:,(i-2)*nq+(1:nq)) = dc(:,(i-2)*nq+(1:nq))-dcdpos_diff*J1;
        dc(:,(i-1)*nq+(1:nq)) = dc(:,(i-1)*nq+(1:nq))+dcdpos_diff*J2;
        pts1_in_B = pts2_in_B;
        J1 = J2;
      end
    end
  end
end