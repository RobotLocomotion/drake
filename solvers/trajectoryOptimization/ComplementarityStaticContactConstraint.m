classdef ComplementarityStaticContactConstraint 
  % enforces the constraint
  % force_normal(i)*(contact_pos(i)-contact_pos(i+1)) = 0
  % contact_pos_x(i)-contact_pos_x(i+1) <= gamma1
  % contact_pos_x(i)-contact_pos_x(i+1) >= -gamma1
  % contact_pos_y(i)-contact_pos_y(i+1) <= gamma2
  % contact_pos_y(i)-contact_pos_y(i+1) >= -gamma2
  % <force_normal(i),gamma1> = 0 (elementwise)
  % <force_normal(i),gamma2> = 0 (elementwise)
  
  properties(SetAccess = protected)
    rb_wrench  % A RigidBodyWrench object
    nlcon
  end
  
  properties(Access = protected)
    A_force
    A_torque
    nq
    num_pts
    num_pt_F
    num_lambda
    dnormal_force_dlambda
    lambda_idx
    q1_idx
    q2_idx
    gamma_idx
  end
  
  methods
    function obj = ComplementarityStaticContactConstraint(rb_wrench,ncp_tol)
      if(nargin<2)
        ncp_tol = 0;
      end
      if(~isa(rb_wrench,'RigidBodyContactWrench'))
        error('Drake:ComplementarityStaticContactConstraint:InvalidArguments','Argument rb_wrench should be a RigidBodyContactWrench object');
      end
      if(~isa(rb_wrench,'LinearFrictionConeWrench') && ~isa(rb_wrench,'FrictionConeWrench'))
        error('Drake:ComplementarityStaticContactConstraint: only support LinearFrictionConeWrench or FrictionConeWrench');
      end
      obj.nq = rb_wrench.robot.getNumPositions();
      obj.num_lambda = rb_wrench.num_pt_F*rb_wrench.num_pts;
      obj.num_pt_F = rb_wrench.num_pt_F;
      obj.rb_wrench = rb_wrench;
      obj.A_force = obj.rb_wrench.force();
      obj.A_torque = obj.rb_wrench.torque();
      obj.num_pts = obj.rb_wrench.num_pts;
      obj.q1_idx = (1:obj.nq)';
      obj.q2_idx = obj.nq+(1:obj.nq)';
      obj.lambda_idx = 2*obj.nq+(1:obj.num_lambda)';
      obj.gamma_idx = 2*obj.nq+obj.num_lambda+(1:obj.num_pts)';
      obj.dnormal_force_dlambda = sparse(reshape(bsxfun(@times,ones(3,1),1:obj.num_pts),[],1),(1:3*obj.num_pts)',obj.rb_wrench.normal_dir(:),obj.num_pts,3*obj.num_pts)*obj.A_force;
      nlcon_name = cell(2*rb_wrench.num_pts,1);
      for i = 1:rb_wrench.num_pts
        nlcon_name{i} = sprintf('%s_ComplementarityStaticContact_pos%d',obj.rb_wrench.body_name,i);
        nlcon_name{i+rb_wrench.num_pts} = sprintf('%s_ComplementarityStaticContact_product%d',obj.rb_wrench.body_name,i);
      end
      obj.nlcon = FunctionHandleConstraint([zeros(rb_wrench.num_pts,1);-ncp_tol*ones(rb_wrench.num_pts,1)],[zeros(rb_wrench.num_pts,1);ncp_tol*ones(rb_wrench.num_pts,1)],2*obj.nq+obj.num_lambda+rb_wrench.num_pts,@(~,~,lambda,gamma,kinsol1,kinsol2) nlconEval(obj,kinsol1,kinsol2,lambda,gamma));
      obj.nlcon = obj.nlcon.setName(nlcon_name);
      
      nlcon_iCfun1 = [reshape(bsxfun(@times,(1:obj.num_pts)',ones(1,numel(obj.rb_wrench.kinematics_chain_idx))),[],1);reshape(bsxfun(@times,(1:obj.num_pts)',ones(1,numel(obj.rb_wrench.kinematics_chain_idx))),[],1);(1:obj.num_pts)'];
      nlcon_jCvar1 = [reshape(bsxfun(@times,ones(obj.num_pts,1),obj.q1_idx(obj.rb_wrench.kinematics_chain_idx)'),[],1);reshape(bsxfun(@times,ones(obj.num_pts,1),obj.q2_idx(obj.rb_wrench.kinematics_chain_idx)'),[],1);obj.gamma_idx];
      nlcon_iCfun2 = [reshape(bsxfun(@times,1:obj.num_pts,ones(obj.num_pt_F,1)),[],1);(1:obj.num_pts)'];
      nlcon_jCvar2 = [obj.lambda_idx;obj.gamma_idx];
      nlcon_iCfun = [nlcon_iCfun1;nlcon_iCfun2+obj.num_pts];
      nlcon_jCvar = [nlcon_jCvar1;nlcon_jCvar2];
      obj.nlcon = obj.nlcon.setSparseStructure(nlcon_iCfun,nlcon_jCvar);
    end
    
    function [nlcon,slack_bcon,num_slack,slack_name] = generateConstraint(obj)
      num_slack = obj.rb_wrench.num_pts;
      slack_name = cell(num_slack,1);
      for i = 1:num_slack
        slack_name{i} = sprintf('%s_ComplementarityStaticContact_slack%d',obj.rb_wrench.body_name,i);
      end
      nlcon = obj.nlcon;
      slack_bcon = BoundingBoxConstraint(zeros(num_slack,1),inf(num_slack,1));
    end
    
    function [c,dc] = nlconEval(obj,kinsol1,kinsol2,lambda,gamma)
      [pos1,dpos1dq1] = obj.rb_wrench.robot.forwardKin(kinsol1,obj.rb_wrench.body,obj.rb_wrench.body_pts,0);
      [pos2,dpos2dq2] = obj.rb_wrench.robot.forwardKin(kinsol2,obj.rb_wrench.body,obj.rb_wrench.body_pts,0);
      pos_diff = pos1-pos2;
      dpos_diff_dq = [dpos1dq1 -dpos2dq2];
      pos_diff_normal = bsxfun(@times,ones(3,1),sum(pos_diff.*obj.rb_wrench.normal_dir,1)).*obj.rb_wrench.normal_dir;
      pos_diff_tangential = pos_diff-pos_diff_normal;
      dpos_diff_tangential_dq = (eye(3*obj.num_pts)-obj.rb_wrench.normal_dir(:)*obj.rb_wrench.normal_dir(:)')*dpos_diff_dq;
      c1 = sum(pos_diff_tangential.^2,1)'-gamma;
      num_vars = 2*obj.nq+obj.num_lambda+obj.num_pts;
      dc1 = zeros(obj.num_pts,num_vars);
      dc1(:,[obj.q1_idx;obj.q2_idx]) = sparse(reshape(bsxfun(@times,ones(3,1),1:obj.num_pts),[],1),(1:3*obj.num_pts)',2*pos_diff_tangential(:),obj.num_pts,3*obj.num_pts)*dpos_diff_tangential_dq;
      dc1(:,obj.gamma_idx) = -eye(obj.num_pts);
      normal_force = reshape(obj.dnormal_force_dlambda*lambda(:),[],1);
      force_scaler = obj.rb_wrench.robot.getMass*9.81;
      c2 = gamma.*normal_force/force_scaler;
      dc2 = zeros(obj.num_pts,num_vars);
      dc2(:,obj.lambda_idx) = diag(gamma)*obj.dnormal_force_dlambda/force_scaler;
      dc2(:,obj.gamma_idx) = diag(normal_force/force_scaler);
      c = [c1;c2];
      dc = [dc1;dc2];
    end
  end
  
    
end