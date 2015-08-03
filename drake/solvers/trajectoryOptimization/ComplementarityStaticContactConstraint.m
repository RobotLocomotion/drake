classdef ComplementarityStaticContactConstraint 
  % enforces the constraint
  % force_normal(i)*(contact_pos(i)-contact_pos(i+1)) = 0
  % By 
  % contact_pos_x(i)-contact_pos_x(i+1) <= gamma1
  % contact_pos_x(i)-contact_pos_x(i+1) >= -gamma1
  % contact_pos_y(i)-contact_pos_y(i+1) <= gamma2
  % contact_pos_y(i)-contact_pos_y(i+1) >= -gamma2
  % <force_normal(i),gamma1> = 0 (elementwise)
  % <force_normal(i),gamma2> = 0 (elementwise)
  % gamma1 >= 0
  % gamma2 >= 0
  
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
    gamma1_idx
    gamma2_idx
    tangent_x_mat
    tangent_y_mat
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
      obj.gamma1_idx = 2*obj.nq+obj.num_lambda+(1:obj.num_pts)';
      obj.gamma2_idx = 2*obj.nq+obj.num_lambda+obj.num_pts+(1:obj.num_pts)';
      obj.dnormal_force_dlambda = sparse(reshape(bsxfun(@times,ones(3,1),1:obj.num_pts),[],1),(1:3*obj.num_pts)',obj.rb_wrench.normal_dir(:),obj.num_pts,3*obj.num_pts)*obj.A_force;
      
      % determine the local tangential directions
      obj.tangent_x_mat = zeros(obj.num_pts,3*obj.num_pts);
      obj.tangent_y_mat = zeros(obj.num_pts,3*obj.num_pts);
      for i = 1:obj.num_pts
        normal_dir = obj.rb_wrench.normal_dir(:,i);
        normal_dir = normal_dir/norm(normal_dir);
        [~,z_axis_ind] = max(sum(cross(bsxfun(@times,normal_dir,ones(1,3)),eye(3)),1));
        z_axis = [0;0;0];
        z_axis(z_axis_ind) = 1;
        x_axis = [0;0;0];
        x_axis(mod(z_axis_ind,3)+1) = 1;
        y_axis = [0;0;0];
        y_axis(mod(z_axis_ind+1,3)+1) = 1;
        rotate_axis = cross(z_axis,normal_dir);
        rotate_angle = asin(norm(rotate_axis));
        rotate_axis = rotate_axis/norm(rotate_axis);
        rotmat = axis2rotmat([rotate_axis;rotate_angle]);
        obj.tangent_x_mat(i,3*(i-1)+(1:3)) = (rotmat*x_axis)';
        obj.tangent_y_mat(i,3*(i-1)+(1:3)) = (rotmat*y_axis)';
      end
      
      nlcon_name = cell(6*rb_wrench.num_pts,1);
      for i = 1:obj.num_pts
        nlcon_name{i} = sprintf('%s_pt%d_contact_pos_x_diff<=gamma1',obj.rb_wrench.body_name,i);
        nlcon_name{i+obj.num_pts} = sprintf('%s_pt%d_contact_pos_x_diff>=-gamma1',obj.rb_wrench.body_name,i);
        nlcon_name{i+2*obj.num_pts} = sprintf('%s_pt%d_contact_pos_y_diff<=gamma2',obj.rb_wrench.body_name,i);
        nlcon_name{i+3*obj.num_pts} = sprintf('%s_pt%d_contact_pos_y_diff>=-gamma2',obj.rb_wrench.body_name,i);
        nlcon_name{i+4*obj.num_pts} = sprintf('%s_pt%d_force x gamma1=0',obj.rb_wrench.body_name,i);
        nlcon_name{i+5*obj.num_pts} = sprintf('%s_pt%d_force x gamma2=0',obj.rb_wrench.body_name,i);
      end
      nlcon_lb = [-inf(obj.num_pts,1);zeros(obj.num_pts,1);-inf(obj.num_pts,1);zeros(obj.num_pts,1);-ncp_tol*ones(obj.num_pts,1);-ncp_tol*ones(obj.num_pts,1)];
      nlcon_ub = [zeros(obj.num_pts,1);inf(obj.num_pts,1);zeros(obj.num_pts,1);inf(obj.num_pts,1);ncp_tol*ones(obj.num_pts,1);ncp_tol*ones(obj.num_pts,1)];
      obj.nlcon = FunctionHandleConstraint(nlcon_lb,nlcon_ub,2*obj.nq+obj.num_lambda+2*obj.num_pts,@(~,~,lambda,gamma,kinsol1,kinsol2) nlconEval(obj,kinsol1,kinsol2,lambda,gamma));
      obj.nlcon = obj.nlcon.setName(nlcon_name);
      
      nlcon_iCfun1 = [reshape(bsxfun(@times,(1:obj.num_pts)',ones(1,numel(obj.rb_wrench.kinematics_chain_idx))),[],1);reshape(bsxfun(@times,(1:obj.num_pts)',ones(1,numel(obj.rb_wrench.kinematics_chain_idx))),[],1);(1:obj.num_pts)'];
      nlcon_jCvar1 = [reshape(bsxfun(@times,ones(obj.num_pts,1),obj.q1_idx(obj.rb_wrench.kinematics_chain_idx)'),[],1);reshape(bsxfun(@times,ones(obj.num_pts,1),obj.q2_idx(obj.rb_wrench.kinematics_chain_idx)'),[],1);obj.gamma1_idx];
      nlcon_iCfun2 = [reshape(bsxfun(@times,(1:obj.num_pts)',ones(1,numel(obj.rb_wrench.kinematics_chain_idx))),[],1);reshape(bsxfun(@times,(1:obj.num_pts)',ones(1,numel(obj.rb_wrench.kinematics_chain_idx))),[],1);(1:obj.num_pts)'];
      nlcon_jCvar2 = [reshape(bsxfun(@times,ones(obj.num_pts,1),obj.q1_idx(obj.rb_wrench.kinematics_chain_idx)'),[],1);reshape(bsxfun(@times,ones(obj.num_pts,1),obj.q2_idx(obj.rb_wrench.kinematics_chain_idx)'),[],1);obj.gamma1_idx];
      nlcon_iCfun3 = [reshape(bsxfun(@times,(1:obj.num_pts)',ones(1,numel(obj.rb_wrench.kinematics_chain_idx))),[],1);reshape(bsxfun(@times,(1:obj.num_pts)',ones(1,numel(obj.rb_wrench.kinematics_chain_idx))),[],1);(1:obj.num_pts)'];
      nlcon_jCvar3 = [reshape(bsxfun(@times,ones(obj.num_pts,1),obj.q1_idx(obj.rb_wrench.kinematics_chain_idx)'),[],1);reshape(bsxfun(@times,ones(obj.num_pts,1),obj.q2_idx(obj.rb_wrench.kinematics_chain_idx)'),[],1);obj.gamma2_idx];
      nlcon_iCfun4 = [reshape(bsxfun(@times,(1:obj.num_pts)',ones(1,numel(obj.rb_wrench.kinematics_chain_idx))),[],1);reshape(bsxfun(@times,(1:obj.num_pts)',ones(1,numel(obj.rb_wrench.kinematics_chain_idx))),[],1);(1:obj.num_pts)'];
      nlcon_jCvar4 = [reshape(bsxfun(@times,ones(obj.num_pts,1),obj.q1_idx(obj.rb_wrench.kinematics_chain_idx)'),[],1);reshape(bsxfun(@times,ones(obj.num_pts,1),obj.q2_idx(obj.rb_wrench.kinematics_chain_idx)'),[],1);obj.gamma2_idx];
      nlcon_iCfun5 = [reshape(bsxfun(@times,1:obj.num_pts,ones(obj.num_pt_F,1)),[],1);(1:obj.num_pts)'];
      nlcon_jCvar5 = [obj.lambda_idx;obj.gamma1_idx];
      nlcon_iCfun6 = [reshape(bsxfun(@times,1:obj.num_pts,ones(obj.num_pt_F,1)),[],1);(1:obj.num_pts)'];
      nlcon_jCvar6 = [obj.lambda_idx;obj.gamma2_idx];
      nlcon_iCfun = [nlcon_iCfun1;nlcon_iCfun2+obj.num_pts;nlcon_iCfun3+2*obj.num_pts;nlcon_iCfun4+3*obj.num_pts;nlcon_iCfun5+4*obj.num_pts;nlcon_iCfun6+5*obj.num_pts];
      nlcon_jCvar = [nlcon_jCvar1;nlcon_jCvar2;nlcon_jCvar3;nlcon_jCvar4;nlcon_jCvar5;nlcon_jCvar6];
      obj.nlcon = obj.nlcon.setSparseStructure(nlcon_iCfun,nlcon_jCvar);
      
      
    end
    
    function [nlcon,slack_bcon,num_slack,slack_name] = generateConstraint(obj)
      num_slack = 2*obj.num_pts;
      slack_name = cell(num_slack,1);
      for i = 1:num_slack
        slack_name{i} = sprintf('%s_ComplementarityStaticContact_slack%d',obj.rb_wrench.body_name,i);
      end
      nlcon = obj.nlcon;
      slack_bcon = BoundingBoxConstraint(zeros(num_slack,1),inf(num_slack,1));
    end
    
    function [c,dc] = nlconEval(obj,kinsol1,kinsol2,lambda,gamma)
      gamma1 = gamma(1:obj.num_pts);
      gamma2 = gamma(obj.num_pts+(1:obj.num_pts));
      [pos1,dpos1dq1] = obj.rb_wrench.robot.forwardKin(kinsol1,obj.rb_wrench.body,obj.rb_wrench.body_pts,0);
      [pos2,dpos2dq2] = obj.rb_wrench.robot.forwardKin(kinsol2,obj.rb_wrench.body,obj.rb_wrench.body_pts,0);
      pos_diff = pos1-pos2;
      dpos_diff_dq = [dpos1dq1 -dpos2dq2];
      pos_diff_x = obj.tangent_x_mat*pos_diff(:);
      pos_diff_y = obj.tangent_y_mat*pos_diff(:);
      dpos_diff_x_dq = obj.tangent_x_mat*dpos_diff_dq;
      dpos_diff_y_dq = obj.tangent_y_mat*dpos_diff_dq;
      c1 = pos_diff_x-gamma1;
      c2 = pos_diff_x+gamma1;
      c3 = pos_diff_y-gamma2;
      c4 = pos_diff_y+gamma2;
      num_vars = 2*obj.nq+obj.num_lambda+2*obj.num_pts;
      dc1 = zeros(obj.num_pts,num_vars);
      dc1(:,[obj.q1_idx;obj.q2_idx]) = dpos_diff_x_dq;
      dc1(:,obj.gamma1_idx) = -eye(obj.num_pts);
      dc2 = zeros(obj.num_pts,num_vars);
      dc2(:,[obj.q1_idx;obj.q2_idx]) = dpos_diff_x_dq;
      dc2(:,obj.gamma1_idx) = eye(obj.num_pts);
      dc3 = zeros(obj.num_pts,num_vars);
      dc3(:,[obj.q1_idx;obj.q2_idx]) = dpos_diff_y_dq;
      dc3(:,obj.gamma2_idx) = -eye(obj.num_pts);
      dc4 = zeros(obj.num_pts,num_vars);
      dc4(:,[obj.q1_idx;obj.q2_idx]) = dpos_diff_y_dq;
      dc4(:,obj.gamma2_idx) = eye(obj.num_pts);
      
      normal_force = reshape(obj.dnormal_force_dlambda*lambda(:),[],1);
      force_scaler = obj.rb_wrench.robot.getMass*9.81;
      c5 = gamma1.*normal_force/force_scaler;
      dc5 = zeros(obj.num_pts,num_vars);
      dc5(:,obj.lambda_idx) = diag(gamma1)*obj.dnormal_force_dlambda/force_scaler;
      dc5(:,obj.gamma1_idx) = diag(normal_force/force_scaler);
      c6 = gamma2.*normal_force/force_scaler;
      dc6 = zeros(obj.num_pts,num_vars);
      dc6(:,obj.lambda_idx) = diag(gamma2)*obj.dnormal_force_dlambda/force_scaler;
      dc6(:,obj.gamma2_idx) = diag(normal_force/force_scaler);
      c = [c1;c2;c3;c4;c5;c6];
      dc = [dc1;dc2;dc3;dc4;dc5;dc6];
    end
  end
  
    
end