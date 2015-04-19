classdef ComDynamicsFullKinematicsPlanner < SimpleDynamicsFullKinematicsPlanner
  % This planner impose the following dynamics constraint
  % kc_com(q) = com at evey t_knot
  % H(:,i) -H(:,i-1) = Hdot(:,i)*dt(i)
  % Hdot(:,i) = sum_j cross(p_contact_j-com(:,i),F_j)
  % com(:,i) - com(:,i-1) = comdot(:,i)*dt(i)
  % comdot(:,i)-comdot(:,i-1) = comddot(:,i)*dt(i)
  % m*comddot(:,i) = sum_j F_j-m*g
  % q(:,i)-q(:,i-1) = v(:,i)*dt(i)
  % A*v(:,i) = H(:,i) where A = robot.centroidalMomentumMatrix
  properties(SetAccess = protected)
    com_inds % A 3 x obj.N matrix. x(com_inds(:,i)) is the com position at i'th knot point
    comdot_inds % A 3 x obj.N matrix. x(comdot_inds(:,i)) is the com velocity at i'th knot point
    comddot_inds % A 3 x obj.N matrix. x(comddot_inds(:,i)) is the com acceleation at i'th knot point
    H_inds % A 3 x obj.N matrix. x(H_inds(:,i)) is the centroidal angular momentum at i'th knot point
    Hdot_inds % A 3 x obj.N matrix. x(Hdot_inds(:,i)) is the rate of centroidal angular momentum at i'th knot point
    torque_multiplier % A positive scaler. 
    ncp_tol; % The tolerance for nonlinear complementarity constraint
  end
  
  methods
    function obj = ComDynamicsFullKinematicsPlanner(robot,N,tf_range,Q_comddot,Qv,Q,q_nom,Q_contact_force,contact_wrench_struct,options)
      % @param Q_comddot  A 3 x 3 matrix. penalize sum_j comddot(:,j)*Q_comddot*comddot(:,j)
      % @param Q  an nq x nq matrix. Add the cost sum_j
      % (q(:,j)-q_nom(:,j))'*Q*(q(:,j)-q_nom(:,j));
      if nargin < 10, options = struct(); end
      if(~isfield(options,'ncp_tol'))
        options.ncp_tol = 0;
      end
      
      plant = SimpleDynamicsDummyPlant(robot.getNumPositions());
      obj = obj@SimpleDynamicsFullKinematicsPlanner(plant,robot,N,tf_range,Q_contact_force,contact_wrench_struct,options);
%       obj = obj.setSolverOptions('snopt','scaleoption',2);
      if(~isfield(options,'torque_multiplier'))
        options.torque_multiplier = obj.robot_mass*obj.g*1;
      else
        if(~isnumeric(options.torque_multiplier) || numel(options.torque_multiplier)~= 1 || options.torque_multiplier<=0)
          error('Drake:ComDynamicsFullKinematicsPlanner:torque_multiplier should be positive scaler');
        end
      end
      
      obj.torque_multiplier = options.torque_multiplier;
      obj.ncp_tol = options.ncp_tol;
      obj.com_inds = obj.num_vars+reshape(1:3*obj.N,3,obj.N);
      obj.comdot_inds = obj.num_vars+3*obj.N+reshape(1:3*obj.N,3,obj.N);
      obj.comddot_inds = obj.num_vars+6*obj.N+reshape(1:3*obj.N,3,obj.N);
      x_name = cell(9*obj.N,1);
      for i = 1:obj.N
        x_name{(i-1)*3+1} = sprintf('com_x[%d]',i);
        x_name{(i-1)*3+2} = sprintf('com_y[%d]',i);
        x_name{(i-1)*3+3} = sprintf('com_z[%d]',i);
        x_name{3*obj.N+(i-1)*3+1} = sprintf('comdot_x[%d]',i);
        x_name{3*obj.N+(i-1)*3+2} = sprintf('comdot_y[%d]',i);
        x_name{3*obj.N+(i-1)*3+3} = sprintf('comdot_z[%d]',i);
        x_name{6*obj.N+(i-1)*3+1} = sprintf('comddot_x[%d]',i);
        x_name{6*obj.N+(i-1)*3+2} = sprintf('comddot_y[%d]',i);
        x_name{6*obj.N+(i-1)*3+3} = sprintf('comddot_z[%d]',i);
      end
      obj = obj.addDecisionVariable(9*obj.N,x_name);
      obj.H_inds = obj.num_vars+reshape(1:3*obj.N,3,obj.N);
      obj.Hdot_inds = obj.num_vars+3*obj.N+reshape(1:3*obj.N,3,obj.N);
      x_name = cell(6*obj.N,1);
      for i = 1:obj.N
        x_name{(i-1)*3+1} = sprintf('H_x[%d]',i);
        x_name{(i-1)*3+2} = sprintf('H_y[%d]',i);
        x_name{(i-1)*3+3} = sprintf('H_z[%d]',i);
        x_name{3*obj.N+(i-1)*3+1} = sprintf('Hdot_x[%d]',i);
        x_name{3*obj.N+(i-1)*3+2} = sprintf('Hdot_y[%d]',i);
        x_name{3*obj.N+(i-1)*3+3} = sprintf('Hdot_z[%d]',i);
      end
      obj = obj.addDecisionVariable(6*obj.N,x_name);
      
      obj = addSimpleDynamicConstraints(obj);

      sizecheck(Q,[obj.nq,obj.nq]);
      if(any(eig(Q)<0))
        error('Drake:ComDynamicsFullKinematicsPlanner:Q should be PSD');
      end
      sizecheck(q_nom,[obj.nq,obj.N]);
      %posture_err_cost = QuadraticSumConstraint(-inf,inf,Q,q_nom);
      %obj = obj.addCost(posture_err_cost,reshape(obj.q_inds,[],1));
      for i = 1:obj.N-1
        q_cost = FunctionHandleConstraint(-inf,inf,1+obj.nq,@(h,q) qCost(Q,q_nom(:,i+1),h,q));
        obj = obj.addCost(q_cost,{obj.h_inds(i);obj.q_inds(:,i+1)});
      end

      sizecheck(Q_comddot,[3,3]);
      if(any(eig(Q_comddot)<0))
        error('Drake:ComDynamicsFullKinematicsPlanner:Q_comddot should be PSD');
      end
      %com_accel_cost = QuadraticSumConstraint(-inf,inf,Q_comddot,zeros(3,obj.N));
      %obj = obj.addCost(com_accel_cost,reshape(obj.comddot_inds,[],1));
      com_accel_cost = FunctionHandleConstraint(-inf,inf,4,@(h,comddot) comAccelCost(Q_comddot,h,comddot));
      for i = 1:obj.N-1
        obj = obj.addCost(com_accel_cost,{obj.h_inds(i);obj.comddot_inds(:,i+1)});
      end
      sizecheck(Qv,[obj.nv,obj.nv]);
      if(any(eig(Qv)<0))
        error('Drake:ComDynamicsFullKinematicsPlanner:Q_v should be PSD');
      end
      %v_cost = QuadraticSumConstraint(-inf,inf,Qv,zeros(obj.nv,obj.N));
      %obj = obj.addCost(v_cost,reshape(obj.v_inds,[],1));
      v_cost = FunctionHandleConstraint(-inf,inf,1+obj.nv,@(h,v) vCost(Qv,h,v));
      for i = 1:obj.N-1
        obj = obj.addCost(v_cost,{obj.h_inds(i);obj.v_inds(:,i+1)});
      end
      
      %e = ones(obj.nq*(obj.N-2),1);
      %second_diff_mat = spdiags([e,-2*e,e],[-obj.nq,0,obj.nq],obj.nq*(obj.N-2),obj.nq*obj.N);
      %Qa = second_diff_mat'*kron(eye(obj.N-2),Qv)*second_diff_mat;
      %obj = obj.addCost(QuadraticConstraint(0,0,Qa,zeros(obj.nq*obj.N,1)),obj.q_inds);;

      function [f,df] = comAccelCost(Q,h,comddot)
        %f = 0.5*h*(comddot'*Q*comddot);
        cQc = comddot'*Q*comddot;
        f = h*cQc;
        df = [cQc, 2*h*comddot'*Q];
      end

      function [f,df] = vCost(Q,h,v)
        %f = h^2*(v'*Q*v);
        %df = [2*h*(v'*Q*v), 2*h^2*v'*Q];
        vQv = v'*Q*v;
        f = h*vQv;
        df = [vQv, 2*h*v'*Q];
      end

      function [f,df] = qCost(Q,q_nom,h,q)
        qQq = ((q-q_nom)'*Q*(q-q_nom));
        f = h*qQq;
        df = [qQq, 2*h*(q-q_nom)'*Q];
      end
    end

    function obj = addConstraint(obj, constraint, varargin)
      if isa(constraint, 'RigidBodyConstraint')
        obj = addRigidBodyConstraint(obj,constraint, varargin{:});
      else
        obj = addConstraint@SimpleDynamicsFullKinematicsPlanner(obj,constraint,varargin{:});
      end
    end
    
    
    
    function obj = addCoMBounds(obj,knot_idx,com_lb,com_ub)
      % @param knot_idx  The indices of the knots on which the com position will be
      % constrained within the bounding box.
      knot_idx = knot_idx(:)';
      num_knots = numel(knot_idx);
      sizecheck(com_lb,[3,num_knots]);
      sizecheck(com_ub,[3,num_knots]);
      obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(com_lb(:),com_ub(:)),reshape(obj.com_inds(:,knot_idx),[],1));
    end
    
    function obj = addRunningCost(obj,running_cost)
    end
    
    function [q,v,h,t,com,comdot,comddot,H,Hdot,lambda,wrench] = parseSolution(obj,x_sol)
      nq = obj.robot.getNumPositions;
      nT = obj.N;
      q = reshape(x_sol(obj.q_inds(:)),nq,nT);
      v = reshape(x_sol(obj.v_inds(:)),nq,nT);
      h = reshape(x_sol(obj.h_inds),1,[]);
      t = cumsum([0 h]);
      com = reshape(x_sol(obj.com_inds),3,[]);
      comdot = reshape(x_sol(obj.comdot_inds),3,[]);
      comddot = reshape(x_sol(obj.comddot_inds),3,[]);
      H = reshape(x_sol(obj.H_inds),3,[]);
      Hdot = reshape(x_sol(obj.Hdot_inds),3,[])*obj.torque_multiplier;
      lambda = cell(length(obj.unique_contact_bodies),1);
      for i = 1:length(obj.unique_contact_bodies)
        lambda{i} = reshape(x_sol(obj.lambda_inds{i}),size(obj.lambda_inds{i},1),[],nT);
      end
      wrench = obj.contactWrench(x_sol);
    end
    
    function obj = addDynamicConstraints(obj)
    end
  end
  
  methods(Access = protected)
    function obj = addSimpleDynamicConstraints(obj)
      obj = addCentroidalDynamicConstraints(obj);
      obj = addSimpleDynamicConstraints@SimpleDynamicsFullKinematicsPlanner(obj);
    end
    
    function obj = addCentroidalDynamicConstraints(obj)
      function [c,dc] = comMatch(kinsol,com)
        [com_q,dcom_q] = obj.robot.getCOM(kinsol);
        c = com_q-com;
        dc = [dcom_q -eye(3)];
      end
      function [c,dc] = angularMomentumMatch(kinsol,v,H)
        [A,dA] = obj.robot.centroidalMomentumMatrix(kinsol);
        c = A(1:3,:)*v-H;
        dc = [[eye(3) zeros(3)]*matGradMult(dA,v) A(1:3,:) -eye(3)];
      end
      
      for i = 1:obj.N
        com_cnstr = FunctionHandleConstraint(zeros(3,1),zeros(3,1),obj.nq+3,@(~,com,kinsol) comMatch(kinsol,com));
        com_cnstr = com_cnstr.setName([{sprintf('com_x(q)=com_x[%d]',i)};{sprintf('com_y(q)=com_y[%d]',i)};{sprintf('com_z(q)=com_z[%d]',i)}]);
        obj = obj.addConstraint(com_cnstr,[{obj.q_inds(:,i)};{obj.com_inds(:,i)}],obj.kinsol_dataind(i));
        H_cnstr = FunctionHandleConstraint(zeros(3,1),zeros(3,1),obj.nq+obj.nv+3,@(~,v,H,kinsol) angularMomentumMatch(kinsol,v,H));
        H_cnstr = H_cnstr.setName([{sprintf('A_x*v=H_x[%d]',i)};{sprintf('A_y*v=H_y[%d]',i)};{sprintf('A_z*v=H_z[%d]',i)}]);
        obj = obj.addConstraint(H_cnstr,[{obj.q_inds(:,i)};{obj.v_inds(:,i)};{obj.H_inds(:,i)}],obj.kinsol_dataind(i));
      end
    end
    
    function obj = addContactDynamicConstraints(obj,knot_idx,contact_wrench_idx,knot_lambda_idx)
      num_knots = numel(knot_idx);
      sizecheck(num_knots,[1,1]);
      num_lambda1 = length(knot_lambda_idx{1});
      
      function [c,dc] = singleTimeDynFun(kinsol,com,lambda,Hdot)
        lambda_count = 0;
        c = Hdot*obj.torque_multiplier;
        dc = zeros(3,obj.nq+3+num_lambda1+3);
        dc(1:3,obj.nq+3+num_lambda1+(1:3)) = eye(3)*obj.torque_multiplier;
        for i = contact_wrench_idx{1}
          num_pts_i = obj.contact_wrench{i}.num_pts;
          num_lambda_i = obj.contact_wrench{i}.num_pt_F*num_pts_i;
          force_i = reshape(A_force{i}*lambda(lambda_count+(1:num_lambda_i)),3,num_pts_i);
          pt_torque_i = reshape(A_torque{i}*lambda(lambda_count+(1:num_lambda_i)),3,num_pts_i);
          [contact_pos_i,dcontact_pos_i_dq] = obj.robot.forwardKin(kinsol,obj.contact_wrench{i}.body,obj.contact_wrench{i}.body_pts,0);
          [torque_i,dtorque_i] = crossSum(contact_pos_i-bsxfun(@times,com,ones(1,num_pts_i)),force_i);
          
          c(1:3) = c(1:3)-torque_i-sum(pt_torque_i,2);
          dc(1:3,1:obj.nq) = dc(1:3,1:obj.nq)-dtorque_i(:,1:num_pts_i*3)*dcontact_pos_i_dq;
          dc(1:3,obj.nq+(1:3)) = dc(1:3,obj.nq+(1:3))+dtorque_i(:,1:num_pts_i*3)*sparse((1:num_pts_i*3)',reshape(bsxfun(@times,[1;2;3],ones(1,num_pts_i)),[],1),ones(3*num_pts_i,1));
          dc(1:3,obj.nq+3+lambda_count+(1:num_lambda_i)) = dc(1:3,obj.nq+3+lambda_count+(1:num_lambda_i))-...
            dtorque_i(:,3*num_pts_i+(1:3*num_pts_i))*A_force{i}-...
            sparse(reshape(bsxfun(@times,(1:3)',ones(1,num_pts_i)),[],1),(1:3*num_pts_i)',ones(3*num_pts_i,1),3,3*num_pts_i)*A_torque{i};
          lambda_count = lambda_count+num_lambda_i;
        end
      end

			function [c,dc] = dtDynFun(h,H_l,H_r,Hdot,com_l,com_r,comdot_l,comdot_r,comddot,q_l,q_r,v)
			  c = zeros(3+3+3+obj.nq,1);
				dc = zeros(3+3+3+obj.nq,1+3+3+3+3+3+3+3+3+2*obj.nq+obj.nv);
				c(1:3) = H_r-H_l-Hdot*h*obj.torque_multiplier;
			  dc(1:3,1) = -Hdot*obj.torque_multiplier;
				dc(1:3,1+(1:9)) = [-eye(3) eye(3) -h*eye(3)*obj.torque_multiplier];
				c(4:6) = com_r-com_l-(comdot_l+comdot_r)*h/2;
				dc(4:6,1) = -(comdot_l+comdot_r)/2;
				dc(4:6,1+9+(1:12)) = [-eye(3) eye(3) -h/2*eye(3) -h/2*eye(3)];
				c(7:9) = comdot_r-comdot_l-comddot*h;
				dc(7:9,1) = -comddot;
				dc(7:9,1+15+(1:9)) = [-eye(3) eye(3) -h*eye(3)];
        c(9+(1:obj.nq)) = q_r-q_l-v*h;
        dc(9+(1:obj.nq),1) = -v;
        dc(9+(1:obj.nq),1+24+(1:2*obj.nq+obj.nv)) = [-eye(obj.nq) eye(obj.nq) -h*eye(obj.nv)];
			end

      if(num_knots == 1)
        % Add the dynamic constraints for a single time
        % Hdot = sum_j cross((contact_pos_j-com),F_j)
        % m*comddot+m*g = sum_j F_j
        A_force_stack = zeros(3,num_lambda1);
        lambda_count_tmp = 0;
        A_force = cell(1,length(contact_wrench_idx{1}));
        A_torque = cell(1,length(contact_wrench_idx{1}));
        joint_idx = [];
        for j = contact_wrench_idx{1}
          num_pts_j = obj.contact_wrench{j}.num_pts;
          num_lambda_j = obj.contact_wrench{j}.num_pt_F*num_pts_j;
          A_force{j} = obj.contact_wrench{j}.force();
          A_torque{j} = obj.contact_wrench{j}.torque();
          A_force_stack(:,lambda_count_tmp+(1:num_lambda_j)) = sparse(reshape(bsxfun(@times,[1;2;3],ones(1,num_pts_j)),[],1),(1:3*num_pts_j)',ones(3*num_pts_j,1))*A_force{j};
          joint_idx_j = obj.contact_wrench{j}.kinematics_chain_idx;
          joint_idx = [joint_idx joint_idx_j];
          lambda_count_tmp = lambda_count_tmp+num_lambda_j;
        end
        joint_idx = unique(joint_idx);
        newton_cnstr = LinearConstraint([0;0;-obj.robot_mass*obj.g],[0;0;-obj.robot_mass*obj.g],[obj.robot_mass*eye(3) -A_force_stack]);
        newton_cnstr = newton_cnstr.setName([{sprintf('F_x=ma_x[%d]',knot_idx)};{sprintf('F_y=ma_y[%d]',knot_idx)};{sprintf('F_z=ma_z[%d]',knot_idx)}]);
        obj = obj.addLinearConstraint(newton_cnstr,[obj.comddot_inds(:,knot_idx);knot_lambda_idx{1}]);
        Hdot_cnstr = FunctionHandleConstraint(zeros(3,1),zeros(3,1),obj.nq+3+num_lambda1+3,@(~,com,lambda,Hdot,kinsol) singleTimeDynFun(kinsol,com,lambda,Hdot));
        Hdot_cnstr = Hdot_cnstr.setName([{sprintf('Hdot_x[%d]=rxF',knot_idx)};{sprintf('Hdot_y[%d]=rxF',knot_idx)};{sprintf('Hdot_z[%d]=rxF',knot_idx)}]);
        Hdot_sparse_pattern = zeros(3,Hdot_cnstr.xdim);
        Hdot_sparse_pattern(:,joint_idx) = 1;
        Hdot_sparse_pattern(:,obj.nq+(1:3+num_lambda1)) = 1;
        Hdot_sparse_pattern(:,obj.nq+3+num_lambda1+(1:3)) = eye(3);
        [Hdot_row,Hdot_col] = find(Hdot_sparse_pattern);
        Hdot_cnstr = Hdot_cnstr.setSparseStructure(Hdot_row,Hdot_col);
        obj = obj.addConstraint(Hdot_cnstr,[{obj.q_inds(:,knot_idx)};{obj.com_inds(:,knot_idx)};knot_lambda_idx;{obj.Hdot_inds(:,knot_idx)}],obj.kinsol_dataind(knot_idx));
      elseif(num_knots == 2)
				h_sparse_pattern = zeros(3+3+3+obj.nq,1+24+2*obj.nq+obj.nv);
        h_sparse_pattern(1:3,1) = ones(3,1);
				h_sparse_pattern(1:3,1+(1:9)) = [eye(3) eye(3) eye(3)];
				h_sparse_pattern(4:6,1) = ones(3,1);
				h_sparse_pattern(4:6,1+9+(1:12)) = [eye(3) eye(3) eye(3) eye(3)];
				h_sparse_pattern(7:9,1) = ones(3,1);
				h_sparse_pattern(7:9,1+15+(1:9)) = [eye(3) eye(3) eye(3)];
        h_sparse_pattern(9+(1:obj.nq),1) = 1;
        h_sparse_pattern(9+(1:obj.nq),1+24+(1:2*obj.nq+obj.nv)) = [eye(obj.nq) eye(obj.nq) eye(obj.nv)];
        h_cnstr = FunctionHandleConstraint(zeros(9+obj.nq,1),zeros(9+obj.nq,1),1+24+2*obj.nq+obj.nv,@dtDynFun);
        cnstr_names = cell(9+obj.nq,1);
        cnstr_names(1:9) = [{sprintf('H_x[%d]-H_x[%d]=Hdot_x[%d]*dt',knot_idx(2),knot_idx(1),knot_idx(2))},...
                       {sprintf('H_y[%d]-H_y[%d]=Hdot_y[%d]*dt',knot_idx(2),knot_idx(1),knot_idx(2))},...
                       {sprintf('H_z[%d]-H_z[%d]=Hdot_z[%d]*dt',knot_idx(2),knot_idx(1),knot_idx(2))},...
                       {sprintf('com_x[%d]-com_x[%d]=(comdot_x[%d]+comdot_x[%d])/2*dt',knot_idx(2),knot_idx(1),knot_idx(1),knot_idx(2))},...
                       {sprintf('com_y[%d]-com_y[%d]=(comdot_y[%d]+comdot_y[%d])/2*dt',knot_idx(2),knot_idx(1),knot_idx(1),knot_idx(2))},...
                       {sprintf('com_z[%d]-com_z[%d]=(comdot_z[%d]+comdot_z[%d])/2*dt',knot_idx(2),knot_idx(1),knot_idx(1),knot_idx(2))},...
                       {sprintf('comdot_x[%d]-comdot_x[%d]=comddot_x[%d]*dt',knot_idx(2),knot_idx(1),knot_idx(2))},...
                       {sprintf('comdot_x[%d]-comdot_y[%d]=comddot_y[%d]*dt',knot_idx(2),knot_idx(1),knot_idx(2))},...
                       {sprintf('comdot_x[%d]-comdot_z[%d]=comddot_z[%d]*dt',knot_idx(2),knot_idx(1),knot_idx(2))}];
        for j = 1:obj.nq
          cnstr_names{9+j} = sprintf('q%d[%d]-q%d[%d]=v%d[%d]',j,knot_idx(2),j,knot_idx(1),j,knot_idx(2));
        end
        h_cnstr = h_cnstr.setName(cnstr_names);
        [h_row,h_col] = find(h_sparse_pattern);
        h_cnstr = h_cnstr.setSparseStructure(h_row,h_col);
        obj = obj.addConstraint(h_cnstr,[{obj.h_inds(knot_idx(1))};{obj.H_inds(:,knot_idx(1))};{obj.H_inds(:,knot_idx(2))};{obj.Hdot_inds(:,knot_idx(2))};...
          {obj.com_inds(:,knot_idx(1))};{obj.com_inds(:,knot_idx(2))};{obj.comdot_inds(:,knot_idx(1))};{obj.comdot_inds(:,knot_idx(2))};...
          {obj.comddot_inds(:,knot_idx(2))};{obj.q_inds(:,knot_idx(1))};{obj.q_inds(:,knot_idx(2))};{obj.v_inds(:,knot_idx(2))}]);
        %%%%
        % add the complementarity contact constraint
        % find out the RigidBodyWrench on the same body and same body_pts
        lambda1_start_idx = 0;
        for wrench_idx1 = contact_wrench_idx{1}
          lambda1_idx = knot_lambda_idx{1}(lambda1_start_idx+(1:obj.contact_wrench{wrench_idx1}.num_pt_F*obj.contact_wrench{wrench_idx1}.num_pts));
          lambda2_start_idx = 0;
          for wrench_idx2 = contact_wrench_idx{2}
            if(obj.contact_wrench{wrench_idx1}.contact_force_type == obj.contact_wrench{wrench_idx2}.contact_force_type &&...
                obj.contact_wrench{wrench_idx1}.body == obj.contact_wrench{wrench_idx2}.body && ...
                obj.contact_wrench{wrench_idx1}.num_pts == obj.contact_wrench{wrench_idx2}.num_pts)
              if(sum(sum((obj.contact_wrench{wrench_idx1}.body_pts-obj.contact_wrench{wrench_idx2}.body_pts).^2,1))<1e-4)
                % Now the two contact wrenches are on the same body, same points with same
                % type of contact forces
                lambda2_idx = knot_lambda_idx{2}(lambda2_start_idx+(1:obj.contact_wrench{wrench_idx2}.num_pt_F*obj.contact_wrench{wrench_idx2}.num_pts));
                if(obj.contact_wrench{wrench_idx1}.complementarity_flag || obj.contact_wrench{wrench_idx2}.complementarity_flag)
                  csc_cnstr = ComplementarityStaticContactConstraint(obj.contact_wrench{wrench_idx1},obj.ncp_tol);
                  [csc_nlcon,slack_bcon,num_csc_slack,csc_slack_name] = csc_cnstr.generateConstraint();
                  csc_cnstr_name = cell(csc_nlcon.num_cnstr,1);
                  for csc_cnstr_idx = 1:csc_nlcon.num_cnstr
                    csc_cnstr_name{csc_cnstr_idx} = sprintf('%s_knot%d&%d',csc_nlcon.name{csc_cnstr_idx},knot_idx(1),knot_idx(2));
                  end
                  csc_nlcon = csc_nlcon.setName(csc_cnstr_name);
                  csc_slack_idx = obj.num_vars+(1:num_csc_slack)';
                  obj = obj.addDecisionVariable(num_csc_slack,csc_slack_name);
                  obj = obj.addBoundingBoxConstraint(slack_bcon,csc_slack_idx);
                  if(obj.contact_wrench{wrench_idx1}.complementarity_flag && obj.contact_wrench{wrench_idx2}.complementarity_flag)
                    lambda_idx = lambda2_idx;
                  elseif(obj.contact_wrench{wrench_idx1}.complementarity_flag)
                    lambda_idx = lambda1_idx;
                  elseif(obj.contact_wrench{wrench_idx2}.complementarity_flag)
                    lambda_idx = lambda2_idx;
                  end
                  obj = obj.addConstraint(csc_nlcon,[{obj.q_inds(:,knot_idx(1))};{obj.q_inds(:,knot_idx(2))};{lambda_idx};{csc_slack_idx}],[obj.kinsol_dataind(knot_idx(1)),obj.kinsol_dataind(knot_idx(2))]);
                end
              end
            end
            lambda2_start_idx = lambda2_start_idx+obj.contact_wrench{wrench_idx2}.num_pt_F*obj.contact_wrench{wrench_idx2}.num_pts;
          end
          lambda1_start_idx = lambda1_start_idx+obj.contact_wrench{wrench_idx1}.num_pt_F*obj.contact_wrench{wrench_idx1}.num_pts;
        end
      end
    end
  end
end
