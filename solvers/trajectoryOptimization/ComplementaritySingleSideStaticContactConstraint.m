classdef ComplementaritySingleSideStaticContactConstraint < DifferentiableConstraint
  % enforce the constraint force*(contact_pos(i)-contact_pos(j)) = 0. This is going to be
  % used in the case when we know at one knot point the body is in contact, but uncertian
  % whether the contact is active or not for the other knot
  properties(SetAccess = protected)
    rb_wrench  % A RigidBodyWrench object
  end
  
  properties(Access = protected)
    A_force
    A_torque
    num_pts
    dwrenchdlambda
  end
  
  methods
    function obj = ComplementaritySingleSideStaticContactConstraint(rb_wrench)
      if(~isa(rb_wrench,'RigidBodyContactWrench'))
        error('Drake:ComplementaritySingleSideStaticContactConstraint:InvalidArguments','Argument rb_wrench should be a RigidBodyContactWrench object');
      end
      nq = rb_wrench.robot.getNumPositions();
      num_lambda = rb_wrench.num_pt_F*rb_wrench.num_pts;
      obj = obj@DifferentiableConstraint(zeros(rb_wrench.num_pts,1),zeros(rb_wrench.num_pts,1),num_lambda+2*nq);
      obj.rb_wrench = rb_wrench;
      obj.name = cell(rb_wrench.num_pts,1);
      for i = 1:rb_wrench.num_pts
        obj.name{i} = sprintf('%s_pt%d_static_contact',obj.rb_wrench.body_name,i);
      end
      obj.A_force = obj.rb_wrench.force();
      obj.A_torque = obj.rb_wrench.torque();
      obj.num_pts = obj.rb_wrench.num_pts;
      obj.iCfun = [reshape(bsxfun(@times,(1:obj.num_pts)',ones(1,numel(obj.rb_wrench.kinematics_chain_idx))),[],1);...
        reshape(bsxfun(@times,(1:obj.num_pts)',ones(1,numel(obj.rb_wrench.kinematics_chain_idx))),[],1);...
        reshape(bsxfun(@times,(1:obj.num_pts),ones(obj.rb_wrench.num_pt_F,1)),[],1)];
      obj.jCvar = [reshape(bsxfun(@times,ones(obj.num_pts,1),obj.rb_wrench.kinematics_chain_idx),[],1);...
        nq+reshape(bsxfun(@times,ones(obj.num_pts,1),obj.rb_wrench.kinematics_chain_idx),[],1);...
        2*nq+(1:obj.rb_wrench.num_pt_F*obj.num_pts)'];
      obj.dwrenchdlambda = [obj.A_force;obj.A_torque];
      row_order_idx = reshape([reshape((1:3*obj.num_pts),3,obj.num_pts);3*obj.num_pts+reshape(1:3*obj.num_pts,3,obj.num_pts)],[],1);
      obj.dwrenchdlambda = obj.dwrenchdlambda(row_order_idx,:);
      
    end
  end
  
  methods(Access = protected)
    function [c,dc] = constraintEval(obj,q1,q2,lambda,kinsol1,kinsol2)
      [pos1,dpos1dq1] = obj.rb_wrench.robot.forwardKin(kinsol1,obj.rb_wrench.body,obj.rb_wrench.body_pts,0);
      [pos2,dpos2dq2] = obj.rb_wrench.robot.forwardKin(kinsol2,obj.rb_wrench.body,obj.rb_wrench.body_pts,0);
      pos_diff = sum((pos1-pos2).^2,1)';
      dpos_diffdq = 2*sparse(reshape(bsxfun(@times,ones(3,1),1:obj.num_pts),[],1),(1:3*obj.num_pts)',reshape(pos1-pos2,[],1),obj.num_pts,3*obj.num_pts)*[dpos1dq1 -dpos2dq2];
      wrench = reshape(obj.dwrenchdlambda*lambda(:),6,obj.num_pts);
      wrench_norm = sum(wrench.^2,1)';
      
      dwrench_norm_dwrench_row = reshape(bsxfun(@times,ones(6,1),1:obj.num_pts),[],1);
      dwrench_norm_dwrench_col = (1:6*obj.num_pts)';
      dwrench_norm_dwrench = 2*sparse(dwrench_norm_dwrench_row,dwrench_norm_dwrench_col,reshape(wrench,[],1),obj.num_pts,6*obj.num_pts);
      dwrench_norm_dlambda = dwrench_norm_dwrench*obj.dwrenchdlambda;
      c = wrench_norm.*pos_diff;
      nq = obj.rb_wrench.robot.getNumPositions();
      dcdq = bsxfun(@times,wrench_norm,ones(1,2*nq)).*dpos_diffdq;
      dcdlambda = bsxfun(@times,pos_diff,ones(1,obj.num_pts*obj.rb_wrench.num_pt_F)).*dwrench_norm_dlambda;
      dc = [dcdq dcdlambda];
    end
  end
end