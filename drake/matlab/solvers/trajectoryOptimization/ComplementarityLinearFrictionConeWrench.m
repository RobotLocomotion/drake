classdef ComplementarityLinearFrictionConeWrench < LinearFrictionConeWrench
  % Implement the slack variable version of complementarity constraint.
  % force >= 0 (bcon)
  % distance - gamma = 0 (nlcon)
  % <force,gamma> = 0 (nlcon)
  % gamma >= 0 (bcon)
  properties(SetAccess = protected)
    phi_handle % A function handle. phi_handle(pt_pos) returns the contact distance at each pt_pos, and their gradient w.r.t q
  end
  
  methods
    function obj = ComplementarityLinearFrictionConeWrench(robot,body,body_pts,FC_edge,phi_handle,ncp_tol)
      if(nargin<6)
        ncp_tol = 0;
      end
      obj = obj@LinearFrictionConeWrench(robot,body,body_pts,FC_edge);
      obj.phi_handle = phi_handle;
      obj.num_slack = obj.num_pts;
      obj.slack_name = cell(obj.num_pts,1);
      for i = 1:obj.num_slack
        obj.slack_name{i} = sprintf('%s_linear_friction_cone_complementarity_slack%d',obj.body_name,i);
      end
      obj.slack_lb = zeros(obj.num_pts,1);
      obj.slack_ub = inf(obj.num_pts,1);
      old_num_wrench_cnstr = obj.num_wrench_constraint;
      obj.num_wrench_constraint = obj.num_wrench_constraint+2*obj.num_pts;
      obj.wrench_cnstr_lb = [obj.wrench_cnstr_lb;zeros(obj.num_pts,1);-ncp_tol*ones(obj.num_pts,1)];
      obj.wrench_cnstr_ub = [obj.wrench_cnstr_ub;zeros(obj.num_pts,1);ncp_tol*ones(obj.num_pts,1)];
      nq = obj.robot.getNumPositions();
      obj.wrench_iCfun = [obj.wrench_iCfun;old_num_wrench_cnstr+reshape(bsxfun(@times,(1:obj.num_pts)',ones(1,nq)),[],1);old_num_wrench_cnstr+(1:obj.num_pts)';...distance-gamma = 0
        old_num_wrench_cnstr+obj.num_pts+reshape(bsxfun(@times,ones(obj.num_pt_F,1),1:obj.num_pts),[],1);old_num_wrench_cnstr+obj.num_pts+(1:obj.num_pts)'];%<force,gamma>=0
      obj.wrench_jCvar = [obj.wrench_jCvar;reshape(bsxfun(@times,ones(obj.num_pts,1),(1:nq)),[],1);nq+obj.num_pt_F*obj.num_pts+(1:obj.num_pts)';...
        nq+(1:obj.num_pt_F*obj.num_pts)';nq+obj.num_pts*obj.num_pt_F+(1:obj.num_pts)'];
      comp_name = cell(2*obj.num_pts,1);
      for i = 1:obj.num_pts
        comp_name{i} = sprintf('%s_pt%d_contact_distance-gamma=0',obj.body_name,i);
        comp_name{obj.num_pts+i} = sprintf('<%s_pt%d_force,gamma>=0',obj.body_name,i);
      end
      obj.wrench_cnstr_name = [obj.wrench_cnstr_name;comp_name];
      obj.complementarity_flag = true;
    end
    
    function [c,dc] = evalWrenchConstraint(obj,kinsol,F,slack)
      gamma = slack;
      [c1,dc1] = evalWrenchConstraint@LinearFrictionConeWrench(obj,kinsol,F,slack);
      [pt_pos,dpt_pos] = obj.robot.forwardKin(kinsol,obj.body,obj.body_pts,0);
      [phi,dphidpos] = obj.phi_handle(pt_pos);
      dphidq = dphidpos*dpt_pos;
      c2 = phi-gamma;
      dc2 = [dphidq zeros(obj.num_pts,obj.num_pt_F*obj.num_pts) -eye(obj.num_pts)];
      sum_F = sum(F,1)';
      c3 = gamma.*sum_F;
      dc3dF = sparse(reshape(bsxfun(@times,ones(obj.num_pt_F,1),1:obj.num_pts),[],1),(1:obj.num_pt_F*obj.num_pts)',reshape(bsxfun(@times,ones(obj.num_pt_F,1),gamma'),[],1),obj.num_pts,obj.num_pt_F*obj.num_pts);
      dc3dgamma = diag(sum_F);
      nq = obj.robot.getNumPositions();
      dc3 = [zeros(obj.num_pts,nq) dc3dF dc3dgamma];
      c = [c1;c2;c3];
      dc = [dc1;dc2;dc3];
    end
  end
  
  methods(Access = protected)
    function lincon = generateWrenchLincon(obj)
      lincon_old = generateWrenchLincon@LinearFrictionConeWrench(obj);
      lincon = LinearConstraint(lincon_old.lb,lincon_old.ub,[lincon_old.A zeros(numel(lincon_old.lb),obj.num_slack)]);
      lincon = lincon.setName(lincon_old.name);
    end
  end
end