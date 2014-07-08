classdef ComplementarityStaticContactConstraint 
  % enforces the constraint
  % (force(i)+force(j))*(contact_pos(i)-contact_pos(j)) = 0
  % force(i)+force(j) = gamma1
  % contact_pos(i)-contact_pos(j) = gamma2
  % <gamma1,gamma2> = 0 (elementwise)
  
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
    dwrenchdlambda
    lambda1_idx
    lambda2_idx
    q1_idx
    q2_idx
    gamma1_idx
    gamma2_idx
  end
  
  methods
    function obj = ComplementarityStaticContactConstraint(rb_wrench)
      if(~isa(rb_wrench,'RigidBodyContactWrench'))
        error('Drake:ComplementarityStaticContactConstraint:InvalidArguments','Argument rb_wrench should be a RigidBodyContactWrench object');
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
      obj.lambda1_idx = 2*obj.nq+(1:obj.num_lambda)';
      obj.lambda2_idx = 2*obj.nq+obj.num_lambda+(1:obj.num_lambda)';
      obj.gamma1_idx = 2*obj.nq+2*obj.num_lambda+(1:obj.num_pts)';
      obj.gamma2_idx = 2*obj.nq+2*obj.num_lambda+obj.num_pts+(1:obj.num_pts)';
      obj.dwrenchdlambda = [obj.A_force;obj.A_torque];
      row_order_idx = reshape([reshape((1:3*obj.num_pts),3,obj.num_pts);3*obj.num_pts+reshape(1:3*obj.num_pts,3,obj.num_pts)],[],1);
      obj.dwrenchdlambda = obj.dwrenchdlambda(row_order_idx,:);
      nlcon_name = cell(3*rb_wrench.num_pts,1);
      for i = 1:rb_wrench.num_pts
        nlcon_name{i} = sprintf('%s_ComplementarityStaticContact_force%d',obj.rb_wrench.body_name,i);
        nlcon_name{i+rb_wrench.num_pts} = sprintf('%s_ComplementarityStaticContact_pos%d',obj.rb_wrench.body_name,i);
        nlcon_name{i+2*rb_wrench.num_pts} = sprintf('%s_ComplementarityStaticContact_product%d',obj.rb_wrench.body_name,i);
      end
      obj.nlcon = FunctionHandleConstraint(zeros(3*rb_wrench.num_pts,1),zeros(3*rb_wrench.num_pts,1),2*obj.num_lambda+2*obj.nq+2*rb_wrench.num_pts,@(~,~,lambda1,lambda2,gamma,kinsol1,kinsol2) nlconEval(obj,kinsol1,kinsol2,lambda1,lambda2,gamma));
      obj.nlcon = obj.nlcon.setName(nlcon_name);
      
      nlcon_iCfun1 = [reshape(bsxfun(@times,ones(obj.num_pt_F,1),1:obj.num_pts),[],1);reshape(bsxfun(@times,ones(obj.num_pt_F,1),1:obj.num_pts),[],1);(1:obj.num_pts)'];
      nlcon_jCvar1 = [obj.lambda1_idx;obj.lambda2_idx;obj.gamma1_idx];
      nlcon_iCfun2 = [reshape(bsxfun(@times,(1:obj.num_pts)',ones(1,numel(obj.rb_wrench.kinematics_chain_idx))),[],1);reshape(bsxfun(@times,(1:obj.num_pts)',ones(1,numel(obj.rb_wrench.kinematics_chain_idx))),[],1);(1:obj.num_pts)'];
      nlcon_jCvar2 = [reshape(bsxfun(@times,ones(obj.num_pts,1),obj.q1_idx(obj.rb_wrench.kinematics_chain_idx)'),[],1);reshape(bsxfun(@times,ones(obj.num_pts,1),obj.q2_idx(obj.rb_wrench.kinematics_chain_idx)'),[],1);obj.gamma2_idx];
      nlcon_iCfun3 = [(1:obj.num_pts)';(1:obj.num_pts)'];
      nlcon_jCvar3 = [obj.gamma1_idx;obj.gamma2_idx];
      nlcon_iCfun = [nlcon_iCfun1;nlcon_iCfun2+obj.num_pts;nlcon_iCfun3+2*obj.num_pts];
      nlcon_jCvar = [nlcon_jCvar1;nlcon_jCvar2;nlcon_jCvar3];
      obj.nlcon = obj.nlcon.setSparseStructure(nlcon_iCfun,nlcon_jCvar);
    end
    
    function [nlcon,slack_bcon,num_slack,slack_name] = generateConstraint(obj)
      num_slack = 2*obj.rb_wrench.num_pts;
      slack_name = cell(num_slack,1);
      for i = 1:num_slack
        slack_name{i} = sprintf('%s_ComplementarityStaticContact_slack%d',obj.rb_wrench.body_name,i);
      end
      nlcon = obj.nlcon;
      slack_bcon = BoundingBoxConstraint(zeros(num_slack,1),inf(num_slack,1));
    end
    
    function [c,dc] = nlconEval(obj,kinsol1,kinsol2,lambda1,lambda2,gamma)
      gamma1 = gamma(1:obj.num_pts);
      gamma2 = gamma(obj.num_pts+(1:obj.num_pts));
      [pos1,dpos1dq1] = obj.rb_wrench.robot.forwardKin(kinsol1,obj.rb_wrench.body,obj.rb_wrench.body_pts,0);
      [pos2,dpos2dq2] = obj.rb_wrench.robot.forwardKin(kinsol2,obj.rb_wrench.body,obj.rb_wrench.body_pts,0);
      pos_diff = sum((pos1-pos2).^2,1)';
      dpos_diffdq = 2*sparse(reshape(bsxfun(@times,ones(3,1),1:obj.num_pts),[],1),(1:3*obj.num_pts)',reshape(pos1-pos2,[],1),obj.num_pts,3*obj.num_pts)*[dpos1dq1 -dpos2dq2];
      wrench1 = reshape(obj.dwrenchdlambda*lambda1(:),6,obj.num_pts);
      wrench2 = reshape(obj.dwrenchdlambda*lambda2(:),6,obj.num_pts);
      
      wrench1_norm = sum(wrench1.^2,1)';
      wrench2_norm = sum(wrench2.^2,1)';
      dwrench_norm_dwrench_row = reshape(bsxfun(@times,ones(6,1),1:obj.num_pts),[],1);
      dwrench_norm_dwrench_col = (1:6*obj.num_pts)';
      dwrench1_norm_dwrench1 = 2*sparse(dwrench_norm_dwrench_row,dwrench_norm_dwrench_col,reshape(wrench1,[],1),obj.num_pts,6*obj.num_pts);
      dwrench2_norm_dwrench2 = 2*sparse(dwrench_norm_dwrench_row,dwrench_norm_dwrench_col,reshape(wrench2,[],1),obj.num_pts,6*obj.num_pts);
      dwrench1_norm_dlambda1 = dwrench1_norm_dwrench1*obj.dwrenchdlambda;
      dwrench2_norm_dlambda2 = dwrench2_norm_dwrench2*obj.dwrenchdlambda;
      wrench_scale_factor = (obj.rb_wrench.robot.getMass*9.81)^2;
      c1 = (wrench1_norm+wrench2_norm)/wrench_scale_factor-gamma1;
      num_vars = 2*obj.nq+2*obj.num_lambda+2*obj.num_pts;
      dc1 = zeros(obj.num_pts,num_vars);
      dc1(:,obj.lambda1_idx) = dwrench1_norm_dlambda1/wrench_scale_factor;
      dc1(:,obj.lambda2_idx) = dwrench2_norm_dlambda2/wrench_scale_factor;
      dc1(:,obj.gamma1_idx) = -eye(obj.num_pts);
      c2 = pos_diff;
      dc2 = zeros(obj.num_pts,num_vars);
      dc2(:,[obj.q1_idx;obj.q2_idx]) = dpos_diffdq;
      c3 = gamma1.*gamma2;
      dc3 = zeros(obj.num_pts,num_vars);
      dc3(:,obj.gamma1_idx) = diag(gamma2);
      dc3(:,obj.gamma2_idx) = diag(gamma1);
      c = [c1;c2;c3];
      dc = [dc1;dc2;dc3];
%       c = (wrench1_norm+wrench2_norm).*pos_diff;
%       nq = obj.rb_wrench.robot.getNumPositions();
%       dcdq = bsxfun(@times,wrench1_norm.*wrench2_norm,ones(1,2*nq)).*dpos_diffdq;
%       dcdlambda1 = bsxfun(@times,wrench2_norm.*pos_diff,ones(1,obj.num_pts*obj.rb_wrench.num_pt_F)).*dwrench1_norm_dlambda1;
%       dcdlambda2 = bsxfun(@times,wrench1_norm.*pos_diff,ones(1,obj.num_pts*obj.rb_wrench.num_pt_F)).*dwrench2_norm_dlambda2;
%       dc = [dcdq dcdlambda1 dcdlambda2];
    end
  end
  
    
end