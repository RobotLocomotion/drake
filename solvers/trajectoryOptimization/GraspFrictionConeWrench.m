classdef GraspFrictionConeWrench < RigidBodyContactWrench
  % When consider hand contact, the usual case is that we have a contact
  % patch on the hand, but the contact normal direction is specified on the
  % object being grasped. This class allows the user to specify a grasp
  % point on the hand, with grasp normal on the object
  properties(SetAccess = protected)
    grasp_object_idx  % The index of the object being grasped
    finger_idx  % The index of the finger of grasping
    finger_grasp_pt % The position of the grasping point on the finger, in finger frame
    grasp_object_FC_axis % A 3 x 1 vector. The normal direction in the GRASP OBJECT frame
    FC_mu % The friction coefficient
    force_normalization_factor   % A positive double scaler. The actual force is force_normalization_factor*force_parameter,
                             % where force parameters are the arguments being constrained
  end
  
  properties(Access = protected)
    cos_theta_square; % theta being the half angle of the cone.
  end
  
  methods
    function obj = GraspFrictionConeWrench(robot,grasp_object_idx, finger_idx, finger_grasp_pt,grasp_object_FC_axis,FC_mu,force_normalization_factor)
      obj = obj@RigidBodyContactWrench(robot,finger_idx,finger_grasp_pt);
      if(~isnumeric(grasp_object_idx) || numel(grasp_object_idx) ~= 1)
        error('Drake:GraspFrictionConeWrench: grasp_object_idx should be a scalar');
      end
      obj.finger_idx = finger_idx;
      obj.finger_grasp_pt = finger_grasp_pt;
      obj.grasp_object_idx = grasp_object_idx;
      mu_size = size(FC_mu);
      if(length(mu_size) == 2 && mu_size(1) == 1 && mu_size(2) == 1)
        FC_mu = FC_mu*ones(1,obj.num_pts);
        mu_size = size(FC_mu);
      end
      if(~isnumeric(FC_mu) || any(FC_mu<0) || length(mu_size) ~= 2 || mu_size(1) ~= 1 || mu_size(2) ~= obj.num_pts)
        error('Drake:FrictionConeWrench: FC_mu should be a non-negative 1 x num_pts array');
      end
      obj.FC_mu = FC_mu;
      axis_size = size(grasp_object_FC_axis);
      if(length(axis_size) == 2 && axis_size(1) == 3 && axis_size(2) == 1)
        grasp_object_FC_axis = bsxfun(@times,grasp_object_FC_axis,ones(1,obj.num_pts));
        axis_size = size(grasp_object_FC_axis);
      end
      if(~isnumeric(grasp_object_FC_axis) || length(axis_size) ~= 2 || axis_size(1) ~= 3 || axis_size(2) ~= obj.num_pts)
        error('Drake:FrictionConeWrench: FC_axis should be a 3 x num_pts array');
      end
      FC_axis_norm = sqrt(sum(grasp_object_FC_axis.^2,1));
      obj.grasp_object_FC_axis = grasp_object_FC_axis./(bsxfun(@times,FC_axis_norm,ones(3,1)));
      if(nargin<7)
        force_normalization_factor = obj.robot.getMass*norm(obj.robot.gravity);
      else
        if(~isnumeric(force_normalization_factor) || numel(force_normalization_factor) ~= 1 || force_normalization_factor<=0)
          error('Drake:RigidBodyConeWrench: force_normalization_factor should be a positive scalar');
        end
      end
      obj.force_normalization_factor = force_normalization_factor;
      obj.cos_theta_square = 1./(obj.FC_mu.^2+ones(1,obj.num_pts));
      obj.num_wrench_constraint = 2*obj.num_pts;
      obj.num_pt_F = 3;
      obj.F_lb = -inf(obj.num_pt_F,obj.num_pts);
      obj.F_ub = inf(obj.num_pt_F,obj.num_pts);
      obj.contact_force_type = RigidBodyContactWrench.GraspFrictionConeType;
      obj.wrench_cnstr_lb = zeros(2*obj.num_pts,1);
      obj.wrench_cnstr_ub = inf(2*obj.num_pts,1);
      obj.wrench_cnstr_name = cell(2*obj.num_pts,1);
      for i = 1:obj.num_pts
        obj.wrench_cnstr_name{i} = sprintf('%s_pt%d_friction_cone',obj.body_name,i);
        obj.wrench_cnstr_name{i+obj.num_pts} = sprintf('%s_pt%d_friction_cone',obj.body_name,i);
      end
      nq = obj.robot.getNumPositions;
      obj.wrench_iCfun = [reshape(bsxfun(@times,(1:obj.num_pts)',ones(1,nq)),[],1);reshape(bsxfun(@times,ones(3,1),1:obj.num_pts),[],1)];
      obj.wrench_iCfun = [obj.wrench_iCfun;obj.wrench_iCfun+obj.num_pts];
      obj.wrench_jCvar = repmat([reshape(bsxfun(@times,ones(obj.num_pts,1),1:nq),[],1);obj.robot.getNumPositions+(1:3*obj.num_pts)'],2,1);
    end
    
    function [c,dc] = evalWrenchConstraint(obj,kinsol,F,slack)
      [grasp_object_pos,dgrasp_object_pos] = forwardKin(obj.robot,kinsol,obj.grasp_object_idx,[0;0;0],2);
      grasp_object_quat = grasp_object_pos(4:7);
      dgrasp_object_quat = dgrasp_object_pos(4:7,:);
      [grasp_object_rotmat,dgrasp_object_rotmat_dquat] = quat2rotmat(grasp_object_quat);
      dgrasp_object_rotmat_dq = dgrasp_object_rotmat_dquat*dgrasp_object_quat;
      F_body = grasp_object_rotmat'*F;
      dgrasp_object_rotmat_transpose_dq = transposeGrad(dgrasp_object_rotmat_dq,[3,3]);
      F_norm_square = sum(F_body.^2,1);
      F_times_axis = sum(F_body.*obj.grasp_object_FC_axis,1);
      c1 = F_times_axis';
      c2 = reshape(F_times_axis.^2-obj.cos_theta_square.*F_norm_square,[],1);
      c = [c1;c2];
      dc1dF = sparse(reshape(bsxfun(@times,ones(3,1),1:obj.num_pts),[],1),(1:3*obj.num_pts)',reshape((obj.grasp_object_FC_axis'*grasp_object_rotmat')',[],1),obj.num_pts,3*obj.num_pts);
      nq = obj.robot.getNumPositions();
      dc1dq = zeros(obj.num_pts,nq);
      dc2dq = zeros(obj.num_pts,nq);
      dc2dF = zeros(obj.num_pts,3*obj.num_pts);
      for i = 1:obj.num_pts
        dc1dq(i,:) = reshape(obj.grasp_object_FC_axis(:,i)*F(:,i)',1,[])*dgrasp_object_rotmat_transpose_dq;
        dc2dF_body = (2*F_times_axis(i)*obj.grasp_object_FC_axis(:,i)'-obj.cos_theta_square(i)*2*F_body(:,i)');
        dc2dF(i,(i-1)*3+(1:3)) = dc2dF_body*grasp_object_rotmat';
        dc2dq(i,:) = reshape(dc2dF_body'*F(:,i)',1,[])*dgrasp_object_rotmat_transpose_dq;
      end
      
      dc = [dc1dq dc1dF;dc2dq dc2dF];
    end
    
    function A = torque(obj)
      A = sparse(3*obj.num_pts,obj.num_pt_F*obj.num_pts);
    end
    
    function A = force(obj)
      A = obj.force_normalization_factor*speye(3*obj.num_pts);
    end 
    
    function [pos,J] = contactPosition(obj,kinsol)
      % Returns the contact positions and its gradient w.r.t q
      % @param kinsol   -- The kinematics tree
      % @retval pos     -- A matrix with 3 rows. pos(:,i) is the i'th contact position
      % @retval J       -- A matrix of size prod(size(pos)) x nq. The gradient of pos
      % w.r.t q
      [pos,J] = forwardKin(obj.robot,kinsol,obj.body,obj.body_pts,0);
    end
  end
  
  methods(Access = protected)
    function lincon = generateWrenchLincon(obj)
      lincon = LinearConstraint([],[],zeros(0,3*obj.num_pts));
    end
  end
end