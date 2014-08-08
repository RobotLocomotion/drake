classdef ComplementarityFrictionConeWrench < FrictionConeWrench
  % Implement the slack variable version of nonlinear function
  % force >= 0 (nlcon)
  % distance - gamma = 0 (nlcon)
  % <force,gamma> = 0 (nlcon)
  % gamma >= 0 (bcon)
  properties
    phi_handle % A function handle. phi_handle(pt_pos) returns the distance from the body contact point to the contact enviroment, and its gradient
  end
  
  methods
    function obj = ComplementarityFrictionConeWrench(robot,body,body_pts,FC_mu,FC_axis,phi_handle,force_normalize_factor,ncp_tol)
      % @param phi_handle  A function handle. pho_handle(pt_pos) returns the distance from each pt_pos to the contact enviroment, and its gradient
      if(nargin<7)
        g = 9.81;
        force_normalize_factor = robot.getMass*g;
      end
      if(nargin<8)
        ncp_tol = 0;
      end
      obj = obj@FrictionConeWrench(robot,body,body_pts,FC_mu,FC_axis,force_normalize_factor);
      obj.phi_handle = phi_handle;
      obj.num_slack = obj.num_pts;
      obj.slack_name = cell(obj.num_pts,1);
      for i = 1:obj.num_slack
        obj.slack_name{i} = sprintf('%s_friction_cone_complementarity_slack%d',obj.body_name,i);
      end
      obj.slack_lb = zeros(obj.num_pts,1);
      obj.slack_ub = inf(obj.num_pts,1);
      old_num_wrench_constraint = obj.num_wrench_constraint;
      obj.num_wrench_constraint = obj.num_wrench_constraint+2*obj.num_pts;
      nq = obj.robot.getNumPositions();
      obj.wrench_iCfun = [obj.wrench_iCfun;reshape(bsxfun(@times,old_num_wrench_constraint+(1:obj.num_pts)',ones(1,nq)),[],1);old_num_wrench_constraint+(1:obj.num_pts)';... % distance-gamma = 0
        old_num_wrench_constraint+obj.num_pts+reshape(bsxfun(@times,ones(3,1),1:obj.num_pts),[],1);old_num_wrench_constraint+obj.num_pts+(1:obj.num_pts)']; % <force,gamma> = 0]
      obj.wrench_jCvar = [obj.wrench_jCvar;reshape(bsxfun(@times,ones(obj.num_pts,1),(1:nq)),[],1);nq+3*obj.num_pts+(1:obj.num_pts)';...
        nq+(1:3*obj.num_pts)';nq+3*obj.num_pts+(1:obj.num_pts)'];
      obj.wrench_cnstr_ub = [obj.wrench_cnstr_ub;zeros(obj.num_pts,1);-ncp_tol*ones(obj.num_pts,1)];
      obj.wrench_cnstr_lb = [obj.wrench_cnstr_lb;zeros(obj.num_pts,1);ncp_tol*ones(obj.num_pts,1)];
      comp_name = cell(2*obj.num_pts,1);
      for i = 1:obj.num_pts
        comp_name{i} = sprintf('%s_pt%d_contact_distance-gamma=0',obj.body_name,i);
        comp_name{obj.num_pts+i} = sprintf('<%s_pt%d_normal_force,gamma>=0',obj.body_name,i);
      end
      obj.wrench_cnstr_name = [obj.wrench_cnstr_name;comp_name];
      obj.complementarity_flag = true;
    end
    
    function [c,dc] = evalWrenchConstraint(obj,kinsol,F,slack)
      gamma = slack;
      [c1,dc1] = evalWrenchConstraint@FrictionConeWrench(obj,kinsol,F,slack);
      dc1 = [dc1 zeros(numel(c1),obj.num_slack)];
      [pt_pos,dpt_pos] = obj.robot.forwardKin(kinsol,obj.body,obj.body_pts,0);
      [phi,dphidpos] = obj.phi_handle(pt_pos);
      dphidq = dphidpos*dpt_pos;
      c2 = phi-gamma;
      dc2 = [dphidq zeros(obj.num_pts,3*obj.num_pts) -eye(obj.num_pts)];
      F_normal = sum(F.*obj.FC_axis,1);
      dF_normaldF = sparse(reshape(bsxfun(@times,ones(3,1),1:obj.num_pts),[],1),(1:3*obj.num_pts)',obj.FC_axis(:),obj.num_pts,3*obj.num_pts);
      c3 =F_normal'.*slack;
      dc3dF = bsxfun(@times,slack,ones(1,3*obj.num_pts)).*dF_normaldF;
      nq = obj.robot.getNumPositions();
      dc3dgamma = diag(F_normal');
      dc3 = [zeros(obj.num_pts,nq) dc3dF dc3dgamma];
      c = [c1;c2;c3];
      dc = [dc1;dc2;dc3];
    end
  end
  
  methods(Access = protected)
    function lincon = generateWrenchLincon(obj)
      lincon_old = generateWrenchLincon@FrictionConeWrench(obj);
      lincon = LinearConstraint(lincon_old.lb,lincon_old.ub,[lincon_old.A zeros(length(lincon_old.lb),obj.num_pts)]);
      lincon = lincon.setName(lincon_old.name);
    end
  end
end